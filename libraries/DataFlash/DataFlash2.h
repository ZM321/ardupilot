/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#pragma once
#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#define FINITE_STATE_MACHINE_NAME   "STAT"
#define FINITE_STATE_MACHINE_LABELS "TimeUS,id,stat"
#define FINITE_STATE_MACHINE_FMT    "QHB"

#define SCHEDULER_TASK_NAME   "SCHE"
#define SCHEDULER_TASK_LABELS "TimeUS,id,name,interval,taken,allowed"
#define SCHEDULER_TASK_FMT    "QBZHII"

#define YCHIOT_UWB_NAME   "YCH"
#define YCHIOT_UWB_LABELS "TimeUS,mask,range0,range1,range2,range3,frequency,x,y,z"
#define YCHIOT_UWB_FMT    "Qiiiiiffff"

#define VELOCITY_GPS_NAME   "GPSV"
#define VELOCITY_GPS_LABELS "TimeUS,vx,vy,vz"
#define VELOCITY_GPS_FMT    "Qfff"

#define PSCX_PID_NAME   "PSCX"
#define PSCX_PID_LABELS "TimeUS,p,i,d,ff"
#define PSCX_PID_FMT    "Qffff"

#define PSCY_PID_NAME   "PSCY"
#define PSCY_PID_LABELS "TimeUS,p,i,d,ff"
#define PSCY_PID_FMT    "Qffff"

#define PSC_ALT_NAME   "ALTC"
#define PSC_ALT_LABELS "TimeUS,TAlt,Alt,TVZ,VZ,TAZ,AZ,TOut,THover,TMot,TBst"
#define PSC_ALT_FMT    "Qffffffffff"

enum FiniteStateMachineID {
	MULTI_COPTER_MOTOR_SPOOL_MODE,
	ALT_HOLD_MODE_STATE,
    ID_NUM,
};

class DataFlash2
{
public:
    DataFlash2(){
		if (_instance != nullptr) {
			AP_HAL::panic("DataFlash2 must be singleton");
		}
		_instance = this;
	}

    // get singleton instance
    static DataFlash2 *instance(void) {
        return _instance;
    }

    void Log_Write_FSM(uint16_t id, uint8_t state);
    void Log_Write_SCH(uint8_t i, const char *fun, uint16_t interval_ticks=0, uint32_t time_taken=0, uint32_t task_time_allowed=0);
    void Log_Write_YCHIOT(int32_t mask, int32_t range0, int32_t range1, int32_t range2, int32_t range3, float frequency, float x, float y, float z);
    void Log_Write_GPSVEL(float vx, float vy, float vz);
    void Log_Write_PID_PSCX(float p, float i, float d, float ff);
    void Log_Write_PID_PSCY(float p, float i, float d, float ff);
    void Log_Write_PSC_ALT(float tp, float p, float tv, float v, float ta, float a, float to, float th, float tm, float tb);

    static const struct AP_Param::GroupInfo var_info[];

private:
    static DataFlash2 *_instance;
    AP_Int8 _bitmask[2];
};
