
#ifndef BUZZER_H
#define BUZZER_H

#include "gd32f1x0.h"

extern uint8_t buzzerFreq;
extern uint8_t buzzerPattern;
extern uint8_t buzzerCount;

void poweronMelody(void);
void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern);
void beepLong(uint8_t freq);
void beepShort(uint8_t freq);
void beepShortMany(uint8_t cnt, int8_t dir);

#endif // BUZZER_H