#include <bitset>
#include <STM32FreeRTOS.h>

extern volatile int rotation[];


class KnobRotator {
private:
    int dir = 0;
    int pre = 0;
    int knobNum;

public:
    KnobRotator(int num) : knobNum(num) {}

    void updateRotation(std::bitset<32> keyInput) {        
        // Read and calculate current state
        int offset = 12 + (3-knobNum)*2;
        int cur = keyInput[offset] + (keyInput[offset+1] << 1);
        int ror = (pre << 2) + cur;
        
        // Update rotation based on condition
        if (ror == 0x01 || ror == 0x07 || ror == 0x08 || ror == 0x0e) {
            dir = 1;
        } else if (ror == 0x02 || ror == 0x04 || ror == 0x0b || ror == 0x0d) {
            dir = -1;
        } else if (ror == 0x03 || ror == 0x06 || ror == 0x09 || ror == 0x0c) {
            dir = dir;
        } else {
            dir = 0;
        }
        // Clamp rotation value
        if (rotation[knobNum] + dir < 0) {
          __atomic_store_n(&rotation[knobNum], 0, __ATOMIC_RELAXED);}
        else if (rotation[knobNum] + dir > 16) {
          __atomic_store_n(&rotation[knobNum], 16, __ATOMIC_RELAXED);}
        else {
          __atomic_add_fetch(&rotation[knobNum], dir, __ATOMIC_RELAXED);
        }
        // Update previous state
        pre = cur;
    }
};