//
// Created by roland on 26.05.2023.
//

#ifndef STM32_W5500_DEBUGGER_H
#define STM32_W5500_DEBUGGER_H

typedef struct {
    uint8_t chan;
} Debugger;

Debugger* Debugger_create(uint8_t chan);
void Debugger_log(Debugger* debugger, const char* format, ...);
void SWO_PrintChar(char c, uint8_t chan);
void SWO_PrintString(const char *s, uint8_t chan);


#endif //STM32_W5500_DEBUGGER_H
