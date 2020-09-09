/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Beacon_YCHIOT.h"
#include <ctype.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash2.h>
#include <AP_Bitmask/AP_Bitmask.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Beacon_YCHIOT::AP_Beacon_YCHIOT(AP_Beacon &frontend, AP_SerialManager &serial_manager)
: AP_Beacon_Backend(frontend)
, get_head(false)
, linebuf_len(0)
{
	uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Beacon, 0);
	if (uart != nullptr) {
		uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Beacon, 0), 256, 256);
	}
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_YCHIOT::healthy()
{
	if( (mask & AP_Beacon_YCHIOT_A0_HEALTHY) == 0 ) return false;
	if( (mask & AP_Beacon_YCHIOT_A1_HEALTHY) == 0 ) return false;
	if( (mask & AP_Beacon_YCHIOT_A2_HEALTHY) == 0 ) return false;
	if( (mask & AP_Beacon_YCHIOT_A3_HEALTHY) == 0 ) return false;

	// healthy if we have parsed a message within the past 300ms
	return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_YCHIOT::update(void)
{
	if (uart == nullptr) {
		return;
	}

	int16_t nbytes = uart->available();
	char    head[2] = {0x6D, 0x63};
	char    temp[2];
	uint8_t i = 0;

	memset(temp, 0, sizeof(temp));

	while (nbytes--) {
		char c = uart->read();

		if(get_head == false){
			switch(i){
			case 0:
			case 1:
				temp[i] = c;
				break;
			default:
				temp[0] = temp[1];
				temp[1] = c;
				break;
			}
			if(memcmp(head, temp, sizeof(temp)) == 0){
				fill_buffer(temp[0], linebuf);
				fill_buffer(temp[1], linebuf);
				get_head = true;
			}
		} else {
			fill_buffer(c, linebuf);
		}
		i++;
	}
}

void AP_Beacon_YCHIOT::fill_buffer(char c, char* buf)
{
	linebuf[linebuf_len++] = c;

	if(linebuf_len == AP_Beacon_YCHIOT_MSG_LEN_MAX){

		linebuf_len = 0;
		get_head = false;

		if(linebuf[2]  == 0x20 &&
				linebuf[5]  == 0x20 &&
				linebuf[14] == 0x20 &&
				linebuf[23] == 0x20 &&
				linebuf[32] == 0x20 &&
				linebuf[41] == 0x20 &&
				linebuf[46] == 0x20 &&
				linebuf[49] == 0x20 &&
				linebuf[58] == 0x20 &&
				linebuf[61] == 0x3A)
		{
			parse_buffer();
		}

	}
}

void AP_Beacon_YCHIOT::parse_buffer()
{
	sscanf(&linebuf[0] ,"%s",mid);
	sscanf(&linebuf[3] ,"%x",&mask);
	sscanf(&linebuf[6] ,"%x",&range[0]);
	sscanf(&linebuf[15],"%x",&range[1]);
	sscanf(&linebuf[24],"%x",&range[2]);
	sscanf(&linebuf[33],"%x",&range[3]);
	sscanf(&linebuf[42],"%x",&nranges);
	sscanf(&linebuf[47],"%x",&rseq);
	sscanf(&linebuf[50],"%x",&debug);
	sscanf(&linebuf[59],"%s",ata);

	vec3d report;
	int count = AP_Bitmask::count_one(mask);
	vec3d anchorArray[4];

	anchorArray[0].x = get_anchor_0_pos().x;
	anchorArray[0].y = get_anchor_0_pos().y;
	anchorArray[0].z = get_anchor_0_pos().z;

	anchorArray[1].x = get_anchor_1_pos().x;
	anchorArray[1].y = get_anchor_1_pos().y;
	anchorArray[1].z = get_anchor_1_pos().z;

	anchorArray[2].x = get_anchor_2_pos().x;
	anchorArray[2].y = get_anchor_2_pos().y;
	anchorArray[2].z = get_anchor_2_pos().z;

	anchorArray[3].x = get_anchor_3_pos().x;
	anchorArray[3].y = get_anchor_3_pos().y;
	anchorArray[3].z = get_anchor_3_pos().z;

	if(mask == 0x0007 || mask == 0x000F){
		float frequency = 1000.0f / (AP_HAL::millis() - last_update_ms);
		last_update_ms = AP_HAL::millis();

		GetLocation(&report, ((count==4) ? 1 : 0), &anchorArray[0], &range[0]);

		static double last_x = report.x;
		static double last_y = report.y;
		static double last_z = report.z;

		if(fabsf(last_x - report.x) > AP_Beacon_YCHIOT_MAX_JUMP_M
		|| is_zero(report.x)){
			report.x = last_x;
		} else {
			last_x = report.x;
		}
		if(fabsf(last_y - report.y) > AP_Beacon_YCHIOT_MAX_JUMP_M
		|| is_zero(report.y)){
			report.y = last_y;
		} else {
			last_y = report.y;
		}
		if(fabsf(last_z - report.z) > AP_Beacon_YCHIOT_MAX_JUMP_M
		|| is_zero(report.z)){
			report.z = last_z;
		} else {
			last_z = report.z;
		}


		//Transform YCHIOT to Ardupilot NED
		Vector3f vehicle_position_NED__m = Vector3f(report.x,
				                                   -report.y,
				                                    report.z);

		// But we are conservative here and use 20cm instead (until MM provides us with a proper accuracy value)
		set_vehicle_position(vehicle_position_NED__m, 0.2f);

		// Write log
		DataFlash2::instance()->Log_Write_YCHIOT(mask, range[0], range[1], range[2], range[3], frequency, report.x, -report.y, report.z);
	}
#if AP_Beacon_YCHIOT_DEBUG
	hal.console->printf( "\n mid %s\n", mid);
	hal.console->printf( "\n mask %02x\n", mask);
	hal.console->printf( "\n range0 %08x\n", range[0]);
	hal.console->printf( "\n range 0-%d 1-%d 2-%d 3-%d\n", range[0], range[1], range[2], range[3]);
	hal.console->printf( "\n nranges %d\n", nranges);
	hal.console->printf( "\n rseq %d\n", rseq);
	hal.console->printf( "\n debug %d\n", debug);
	hal.console->printf( "\n ata %s\n", ata);
#endif
}
