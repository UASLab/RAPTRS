#ifndef FRSKY_HXX_
#define FRSKY_HXX_
#include <stdint.h>

void BifrostSetup();
void BifrostSend(float as, uint8_t id, float v, bool soc, uint8_t sel, float ext);

#endif
