#define WATCHES

#ifdef WATCHES
#ifndef SRC_UTILS_DBG_H_
#define SRC_UTILS_DBG_H_

#include <stdint.h>
#include "key_states.h"

/*
 When a key '~' to '6' is pressed, corresponding slot (identified by numbers 0-6) is activated.
 This means that, corresponding watched value is shown on the LED display) and then updated in certain intervals.

 Numbers are outputted in an exponent notation in form '[0A-Z]' + '[0-9][0-9]' where letter denotes added magnitude (A = *10^1, B = *10^2...).
 Letters are used for magnitude so that brain is not confused by seeing three digit numbers.

*/

// This hook is to be placed in usb_report_updater and to be called whenever a key is activated (i.e., on key-down event).
#define WATCH_TRIGGER(STATE) TriggerWatch(STATE);

// When placed into the code, time between calls to this macro is being watched in slot N.
#define WATCH_TIME(N) if(CurrentWatch == N) { WatchTime(N); }

// Watches value V in slot N.
#define WATCH_VALUE(V, N) if(CurrentWatch == N) { WatchValue(V, N); }

// Watches value V in slot N.
#define WATCH_VALUE_MIN(V, N) if(CurrentWatch == N) { WatchValueMin(V, N); }

// Watches value V in slot N.
#define WATCH_VALUE_MAX(V, N) if(CurrentWatch == N) { WatchValueMax(V, N); }

// Watches string V in slot N.
#define WATCH_STRING(V, N) if(CurrentWatch == N) { WatchString(V, N); }

extern uint8_t CurrentWatch;

void TriggerWatch(key_state_t *keyState);
void WatchTime(uint8_t n);
void WatchValue(int v, uint8_t n);
void WatchValueMin(int v, uint8_t n);
void WatchValueMax(int v, uint8_t n);
void WatchString(char const * v, uint8_t n);
void ShowNumberMag(int a);

#endif /* SRC_UTILS_DBG_H_ */

#else

#define WATCH_TRIGGER(N)
#define WATCH_TIME(N)
#define WATCH_VALUE(V, N)
#define WATCH_VALUE_MIN(V, N)
#define WATCH_VALUE_MAX(V, N)

#endif
