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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_OpenMV {
public:
    AP_OpenMV();

    /* Do not allow copies */
    AP_OpenMV(const AP_OpenMV &other) = delete;
    AP_OpenMV &operator=(const AP_OpenMV&) = delete;

    // init - perform required initialisation
    void init(const AP_SerialManager &serial_manager);  //初始化外设

    // update flight control mode. The control mode is vehicle type specific
    bool update(void);  //定时更新外设数据

    /*
     * 定义了两个变量，cx：openmv里的x轴坐标，cy：openmv里的y轴坐标
     * */
    uint8_t cx;
    uint8_t cy;

    //last_frame_ms：最后一次收到openmv传过来帧的时间，单位是毫秒
    uint32_t last_frame_ms;

private:
    //openmv接在飞控哪个串口上面
    AP_HAL::UARTDriver *_port;                  // UART used to send data to receiver
    
    //解析帧的步骤，解析帧走到哪一步了
    uint8_t _step;
    
    //两个临时变量，用来存储接收到的x、y的坐标
    uint8_t _cx_temp;
    uint8_t _cy_temp;
    
};
