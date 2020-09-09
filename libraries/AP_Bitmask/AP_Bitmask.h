#pragma once
#include <stdint.h>
#include <string.h>

class AP_Bitmask
{
public:
	AP_Bitmask();
	virtual ~AP_Bitmask();

    // return how many true bits in n
    static uint8_t  count_one(uint32_t n);

    /** occupy the correct bit of id (set bit true)
     *  return 0: ok
     *  return 1: id < 1;
     *  return 2: id > 256;
     */
    uint8_t  occupy_bit(int16_t id);

    /** clear the correct bit of id (set bit false)
     *  return 0: ok
     *  return 1: id < 1;
     *  return 2: id > 256;
     */
    uint8_t  clear_bit(int16_t id);

    /** check if the correct bit of id have been occupied
     *  return 0: the bit is empty
     *  return 1: id < 1
     *  return 2: id > 256
     *  return 3: the bit have been occupied
     */
    uint8_t  check_bit(int16_t id);

    // return how many bits have been occupied
    uint8_t  count_occupied_bits(void);

    // return bits value
    uint32_t get_bits(uint8_t  i) const { return _bits[i]; }

    // return explicit id in id queue
    int16_t  get_id   (int16_t i)  const { return _id_queue[i]; }

private:
    uint32_t _bits[8];
    int16_t  _id_queue[256];
};
