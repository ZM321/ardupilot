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

//
//  MAVLINK GPS driver
//
#include "AP_GPS_YCHIOT.h"
#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Beacon/AP_Beacon_Backend.h>
#include <AP_Math/AP_Math.h>
#include <DataFlash/DataFlash2.h>

extern const AP_HAL::HAL& hal;

AP_GPS_YCHIOT::AP_GPS_YCHIOT(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{

}

// Reading does nothing in this class; we simply return whether or not
// the latest reading has been consumed.  By calling this function we assume
// the caller is consuming the new data;
bool AP_GPS_YCHIOT::read(void)
{
	if(AP_Beacon::get_singleton()->get_backend()->healthy()){
		Location origin_loc;
		Vector3f position;
		float accuracy_estimate;

		state.have_speed_accuracy = false;
		state.speed_accuracy = 0.0f;
		state.velocity.z = 0.0f;
		state.horizontal_accuracy = 0.0f;
		state.vertical_accuracy = 0.0f;
		state.num_sats = 30;
		state.have_horizontal_accuracy = false;
		state.have_vertical_accuracy = false;
		state.have_vertical_velocity = false;
		state.hdop = 10;
		state.vdop = 20;
		state.status = AP_GPS::GPS_OK_FIX_3D;
		state.last_gps_time_ms = AP_HAL::millis();

		AP_Beacon::get_singleton()->get_origin(origin_loc);
		AP_Beacon::get_singleton()->get_vehicle_position_ned(position, accuracy_estimate);

		state.location.lat = origin_loc.lat + ((double)position.x * (double)LOCATION_SCALING_FACTOR_INV);
		state.location.lng = origin_loc.lng + ((double)position.y * (double)LOCATION_SCALING_FACTOR_INV) / (double)longitude_scale(origin_loc);
		state.location.alt = position.z * 100.0f;

		if(fabsf((double)position.x - _last_x_m) > 0.03f ||
		   fabsf((double)position.y - _last_y_m) > 0.03f)
		{
			double pass_time_s_inv = (double)1000.0f / (double)(AP_HAL::millis() - _last_pos_time);

			state.velocity.x = ((double)position.x - _last_x_m) * pass_time_s_inv;
			state.velocity.y = ((double)position.y - _last_y_m) * pass_time_s_inv;
			state.ground_speed = sqrtf(sq(state.velocity.x) + sq(state.velocity.y));
			state.have_speed_accuracy = true;
			state.speed_accuracy = 0.5f;

			_last_x_m      = (double)position.x;
			_last_y_m      = (double)position.y;
			_last_pos_time = AP_HAL::millis();
		}
		state.velocity.z = 0.0f;
		state.horizontal_accuracy = 0.0f;
		state.vertical_accuracy = 0.0f;
		state.num_sats = 30;
		state.have_horizontal_accuracy = false;
		state.have_vertical_accuracy = false;
		state.have_vertical_velocity = false;
		state.hdop = 10;
		state.vdop = 20;
		state.status = AP_GPS::GPS_OK_FIX_3D;
		state.last_gps_time_ms = AP_HAL::millis();

#if 0
		hal.console->printf("\n");
		hal.console->printf("x: %d\n", state.location.lat);
		hal.console->printf("y: %d\n", state.location.lng);
		hal.console->printf("z: %.4f\n", position.z);
#endif
		return true;
	}

    return false;
}

