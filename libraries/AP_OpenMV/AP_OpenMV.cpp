/*

   Inspired by work done here https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>

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

/* 
   OpenMV library
*/

#define AP_SERIALMANAGER_OPEN_MV_BAUD       115200
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_RX      64
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_TX      64

#include "AP_OpenMV.h"

extern const AP_HAL::HAL& hal;

//constructor  构造函数
AP_OpenMV::AP_OpenMV(void)
{
    _port = NULL;   //openmv串口默认对应是空的
    _step = 0;      //默认解析帧的步骤是0
}

/*
 * init - openmv驱动初始化的过程
 * 查找飞控哪个串口是设置成连接openmv的--关闭流控制--打开串口并开始运行
 */
void AP_OpenMV::init(const AP_SerialManager &serial_manager)
{
    /*
     * 从串口管理器（serial_manager）中去查找串口（find_serial），串口类别是SerialProtocol_OPEN_MV，把查找到的串口赋值给port，
     * 如果port被赋值完后还不是空，那么这个判断就生效，说明已经找到了一个被指派为跟openmv连接的串口
     * 然后开始设置，把flow串口给关掉
     * 然后打开这个openmv串口，begin
     * */
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OPEN_MV, 0))) {
       _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
       //initialise uart
       _port->begin(AP_SERIALMANAGER_OPEN_MV_BAUD, AP_SERIALMANAGER_OPENMV_BUFSIZE_RX, AP_SERIALMANAGER_OPENMV_BUFSIZE_TX);
    }

}

//openmv更新程序  --用来解析openmv帧的
bool AP_OpenMV::update()
{
    if(_port == NULL)   //第一步：先判断接口是否为空，如果是空立即返回错误
        return false;
    
    //如果不为空，将进行下面的操作
    int16_t numc = _port->available();  //读取对应的串口里面已经收到多少个字节数据个数，把这个个数赋给临时变量numc
    uint8_t data;                       //定义一个临时变量data，表示一个字符
    uint8_t checksum = 0;
    
    //然后一个一个从串口接收缓存里面读取这些数据，读取几次-就是读取numc个数
    /*
     * 因为驱动是定时调用的，比如10ms调用一次，这10ms期间可能收到一帧数，比如说收到5个字节的数，
     * 那么就会调用5次，for循环5次，挨个的读取出缓存里面的数据
     * */
    for (int16_t i = 0; i < numc; i++) {
        data = _port->read();

        switch(_step) {
        case 0:
            if(data == 0xA5)
                _step = 1;
            break;

        case 1:
            if(data == 0x5A)
                _step = 2;
            else
                _step = 0;
            break;

        case 2:
            _cx_temp = data;
            _step = 3;
            break;

        case 3:
            _cy_temp = data;
            _step = 4;
            break;

        case 4:
            _step = 0;
            checksum = _cx_temp + _cy_temp;
            if(checksum == data) {
                cx = _cx_temp;
                cy = _cy_temp;
                last_frame_ms = AP_HAL::millis();
                return true;
            }
            break;

        default:
            _step = 0;
        }
    }
    
    return false;
}


