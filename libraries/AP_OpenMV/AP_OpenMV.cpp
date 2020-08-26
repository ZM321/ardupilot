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

//constructor  ���캯��
AP_OpenMV::AP_OpenMV(void)
{
    _port = NULL;   //openmv����Ĭ�϶�Ӧ�ǿյ�
    _step = 0;      //Ĭ�Ͻ���֡�Ĳ�����0
}

/*
 * init - openmv������ʼ���Ĺ���
 * ���ҷɿ��ĸ����������ó�����openmv��--�ر�������--�򿪴��ڲ���ʼ����
 */
void AP_OpenMV::init(const AP_SerialManager &serial_manager)
{
    /*
     * �Ӵ��ڹ�������serial_manager����ȥ���Ҵ��ڣ�find_serial�������������SerialProtocol_OPEN_MV���Ѳ��ҵ��Ĵ��ڸ�ֵ��port��
     * ���port����ֵ��󻹲��ǿգ���ô����жϾ���Ч��˵���Ѿ��ҵ���һ����ָ��Ϊ��openmv���ӵĴ���
     * Ȼ��ʼ���ã���flow���ڸ��ص�
     * Ȼ������openmv���ڣ�begin
     * */
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OPEN_MV, 0))) {
       _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
       //initialise uart
       _port->begin(AP_SERIALMANAGER_OPEN_MV_BAUD, AP_SERIALMANAGER_OPENMV_BUFSIZE_RX, AP_SERIALMANAGER_OPENMV_BUFSIZE_TX);
    }

}

//openmv���³���  --��������openmv֡��
bool AP_OpenMV::update()
{
    if(_port == NULL)   //��һ�������жϽӿ��Ƿ�Ϊ�գ�����ǿ��������ش���
        return false;
    
    //�����Ϊ�գ�����������Ĳ���
    int16_t numc = _port->available();  //��ȡ��Ӧ�Ĵ��������Ѿ��յ����ٸ��ֽ����ݸ��������������������ʱ����numc
    uint8_t data;                       //����һ����ʱ����data����ʾһ���ַ�
    uint8_t checksum = 0;
    
    //Ȼ��һ��һ���Ӵ��ڽ��ջ��������ȡ��Щ���ݣ���ȡ����-���Ƕ�ȡnumc����
    /*
     * ��Ϊ�����Ƕ�ʱ���õģ�����10ms����һ�Σ���10ms�ڼ�����յ�һ֡��������˵�յ�5���ֽڵ�����
     * ��ô�ͻ����5�Σ�forѭ��5�Σ������Ķ�ȡ���������������
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


