//
// Created by roland on 26.05.2023.
//

#ifndef STM32_W5500_STARTTELNETTASK_H
#define STM32_W5500_STARTTELNETTASK_H

#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "socket.h"

#define TELNET_port 23


int TELNET_result = 0;

typedef enum {
    TELNET_INIT = 0,
    TELNET_SOCKET,
    TELNET_BIND,
    TELNET_LISTEN,
    TELNET_IDLE,
    TELNET_READ,
    TELNET_RECEIVED,
    TELNET_CLOSE,
    TELNET_DISABLED,
    TELNET_ERROR,
    TELNET_UNKNOWN
} TELNET_state_typedef;

TELNET_state_typedef TELNET_state = TELNET_INIT;
TELNET_state_typedef TELNET_state_last = TELNET_UNKNOWN;


char TELNET_inputstring[1024] = {0};
uint16_t TELNET_inputstring_pos = 0;
uint8_t TELNET_buffer[1] = {0};
ssize_t TELNET_recv_result = -1;

uint32_t TELNET_wait = 0;
uint32_t TELNET_wait_last = 0;



uint8_t TELNET_socket_status = 0;
uint8_t TELNET_socket_status_last = 0xFF;


#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))

#endif //STM32_W5500_STARTTELNETTASK_H
