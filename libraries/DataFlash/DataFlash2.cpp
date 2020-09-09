#include <DataFlash/DataFlash.h>
#include "DataFlash2.h"

DataFlash2 *DataFlash2::_instance;

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo DataFlash2::var_info[] = {
	    // @Param: _BITMASK
	    // @DisplayName: DataFlash2 log bitmask
	    // @Bitmask: 0:STAT,1:SCHE,2:YCH,3:GPSV,4:PSCX,5:PSCY,6:ALTC
	    // @User: Advanced
    AP_GROUPINFO("_BITMASK",    0, DataFlash2, _bitmask[0], 0xEF),

    AP_GROUPEND
};

void DataFlash2::Log_Write_FSM(uint16_t id, uint8_t state)
{
	if ( (_bitmask[0] & (0x01 << 0)) == 0 ) return;
	DataFlash_Class::instance()->Log_Write(FINITE_STATE_MACHINE_NAME,
			                               FINITE_STATE_MACHINE_LABELS,
										   FINITE_STATE_MACHINE_FMT,
						                   AP_HAL::micros64(),
						                   id,
						                   state);
}

void DataFlash2::Log_Write_SCH(uint8_t i, const char *fun, uint16_t interval_ticks, uint32_t time_taken, uint32_t task_time_allowed)
{
	if ( (_bitmask[0] & (0x01 << 1)) == 0 ) return;
	char function[64] = {};
	strncpy(function, fun, sizeof(function));
	DataFlash_Class::instance()->Log_Write(SCHEDULER_TASK_NAME,
			                               SCHEDULER_TASK_LABELS,
			                               SCHEDULER_TASK_FMT,
						                   AP_HAL::micros64(),
						                   i,
										   function,
										   interval_ticks,
										   time_taken,
										   task_time_allowed);
}

void DataFlash2::Log_Write_YCHIOT(int32_t mask, int32_t range0, int32_t range1, int32_t range2, int32_t range3, float frequency, float x, float y, float z)
{
	if ( (_bitmask[0] & (0x01 << 2)) == 0 ) return;
	DataFlash_Class::instance()->Log_Write(YCHIOT_UWB_NAME,
			                               YCHIOT_UWB_LABELS,
			                               YCHIOT_UWB_FMT,
						                   AP_HAL::micros64(),
						                   mask,
										   range0,
										   range1,
										   range2,
										   range3,
										   frequency,
										   x,
										   y,
										   z);
}

void DataFlash2::Log_Write_GPSVEL(float vx, float vy, float vz)
{
	if ( (_bitmask[0] & (0x01 << 3)) == 0 ) return;
	DataFlash_Class::instance()->Log_Write(VELOCITY_GPS_NAME,
			                               VELOCITY_GPS_LABELS,
			                               VELOCITY_GPS_FMT,
							               AP_HAL::micros64(),
							               vx,
										   vy,
										   vz);
}

void DataFlash2::Log_Write_PID_PSCX(float p, float i, float d, float ff)
{
	if ( (_bitmask[0] & (0x01 << 4)) == 0 ) return;
	DataFlash_Class::instance()->Log_Write(PSCX_PID_NAME,
			                               PSCX_PID_LABELS,
			                               PSCX_PID_FMT,
							               AP_HAL::micros64(),
							               p,
										   i,
										   d,
										   ff);
}

void DataFlash2::Log_Write_PID_PSCY(float p, float i, float d, float ff)
{
	if ( (_bitmask[0] & (0x01 << 5)) == 0 ) return;
	DataFlash_Class::instance()->Log_Write(PSCY_PID_NAME,
			                               PSCY_PID_LABELS,
			                               PSCY_PID_FMT,
							               AP_HAL::micros64(),
							               p,
										   i,
										   d,
										   ff);
}

void DataFlash2::Log_Write_PSC_ALT(float tp, float p, float tv, float v, float ta, float a, float to, float th, float tm, float tb)
{
	if ( (_bitmask[0] & (0x01 << 6)) == 0 ) return;
	DataFlash_Class::instance()->Log_Write(PSC_ALT_NAME,
			                               PSC_ALT_LABELS,
										   PSC_ALT_FMT,
							               AP_HAL::micros64(),
							               tp,
										   p,
										   tv,
										   v,
										   ta,
										   a,
										   to,
										   th,
										   tm,
										   tb);
}
