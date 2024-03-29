#include "utils_dbg.h"

#ifdef WATCHES

#include "led_display.h"
#include "timer.h"
#include "key_states.h"
#include <limits.h>


uint8_t CurrentWatch = 0;

static uint32_t lastWatch = 0;
static uint32_t watchInterval = 500;

void TriggerWatch(key_state_t *keyState)
{
    int16_t key = (keyState - &KeyStates[SlotId_LeftKeyboardHalf][0]);
    if(0 <= key && key <= 7)
    {
        //set the LED value to RES until next update occurs.
        LedDisplay_SetText(3, "RES");
        CurrentWatch = key;
    }
}

void WatchTime(uint8_t n)
{
    static uint32_t lastUpdate = 0;
    if(CurrentTime - lastWatch > watchInterval) {
        ShowNumberMag(CurrentTime - lastUpdate);
        lastWatch = CurrentTime;
    }
    lastUpdate = CurrentTime;
}

void WatchValue(int v, uint8_t n)
{
    if(CurrentTime - lastWatch > watchInterval) {
        ShowNumberMag(v);
        lastWatch = CurrentTime;
    }
}

void WatchValueMin(int v, uint8_t n)
{
    static int m = 0;

    if(v < m) {
        m = v;
    }

    if(CurrentTime - lastWatch > watchInterval) {
        ShowNumberMag(m);
        lastWatch = CurrentTime;
        m = INT_MAX;
    }
}

void WatchValueMax(int v, uint8_t n)
{
    static int m = 0;

    if(v > m) {
        m = v;
    }

    if(CurrentTime - lastWatch > watchInterval) {
        ShowNumberMag(m);
        lastWatch = CurrentTime;
        m = INT_MIN;
    }
}

void WatchString(char const * v, uint8_t n)
{
    if(CurrentTime - lastWatch > watchInterval) {
        LedDisplay_SetText(3,  v);
        lastWatch = CurrentTime;
    }
}


void ShowNumberMag(int a) {
    char b[3];
    int mag = 0;
    int num = a;
    if(num < 0) {
        LedDisplay_SetText(3,  "NEG");
    } else {
        if(num < 1000) {
            b[0] = '0' + num/100;
            b[1] = '0' + num%100/10;
            b[2] = '0' + num%10;
        } else {
            while(num >= 100) {
                mag++;
                num /= 10;
            }
            b[0] = mag == 0 ? '0' : ('A' - 1 + mag);
            b[1] = '0' + num/10;
            b[2] = '0' + num%10;
        }
        LedDisplay_SetText(3,  b);
    }
}

#endif
