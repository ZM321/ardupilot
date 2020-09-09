#include "AP_Bitmask.h"

AP_Bitmask::AP_Bitmask()
{
	memset(_bits   , 0, sizeof(uint32_t)*8);
	memset(_id_queue, 0, sizeof(int16_t) *256);
}

AP_Bitmask::~AP_Bitmask()
{

}

uint8_t AP_Bitmask::count_one(uint32_t n)
{
    n = ((n & 0xAAAAAAAA) >> 1 ) + (n & 0x55555555);
    n = ((n & 0xCCCCCCCC) >> 2 ) + (n & 0x33333333);
    n = ((n & 0xF0F0F0F0) >> 4 ) + (n & 0x0F0F0F0F);
    n = ((n & 0xFF00FF00) >> 8 ) + (n & 0x00FF00FF);
    n = ((n & 0xFFFF0000) >> 16) + (n & 0x0000FFFF);

    return (uint8_t)n;
}

uint8_t AP_Bitmask::occupy_bit(int16_t id)
{
	// check id
	if(id < 1)   return 1;
	if(id > 256) return 2;

	// occupy bit
	_bits[(id-1)/32] |= (1UL<<((id-1)%32));

	// put id into id queue
	_id_queue[count_occupied_bits()-1] = id;

	return 0;
}

uint8_t AP_Bitmask::clear_bit(int16_t id)
{
	uint8_t temp_i = 0;
	uint8_t count_bits = count_occupied_bits();

	// check id
	if(id < 1)   return 1;
	if(id > 256) return 2;

	// remove id from id queue
	for (uint8_t i=0; i<count_bits; i++){
		if(_id_queue[i] == id){
			temp_i = i;
		    break;
		}
	}
	uint8_t remove_num = count_bits-temp_i-1;
	int16_t id_queue_remove[remove_num];
	memcpy(id_queue_remove, &_id_queue[temp_i+1], sizeof(_id_queue[0])*remove_num);
	memcpy(&_id_queue[temp_i], id_queue_remove, sizeof(_id_queue[0])*remove_num);
	_id_queue[count_bits-1] = 0;

	// clear bit
	_bits[(id-1)/32] &= ~(1UL<<((id-1)%32));

	return 0;
}

uint8_t AP_Bitmask::check_bit(int16_t id)
{
	// check id
	if(id < 1)   return 1;
	if(id > 256) return 2;

	if(_bits[(id-1)/32] & (1UL<<((id-1)%32))){
		return 3;
	}

	return 0;
}

uint8_t AP_Bitmask::count_occupied_bits(void)
{
	uint8_t accumulate = 0;

	for(uint8_t i=0; i<8; i++){
		accumulate += count_one(_bits[i]);
	}

	return accumulate;
}
