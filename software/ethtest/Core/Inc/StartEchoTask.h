//
// Created by roland on 13.10.2023.
//

#ifndef ETHTEST_STARTECHOTASK_H
#define ETHTEST_STARTECHOTASK_H

#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sockets.h"

#define ECHO_port 7


typedef enum {
    ECHO_INIT = 0,
    ECHO_SOCKET,
    ECHO_BIND,
    ECHO_LISTEN,
    ECHO_IDLE,
    ECHO_READ,
    ECHO_RECEIVED,
    ECHO_CLOSE,
    ECHO_DISABLED,
    ECHO_ERROR,
    ECHO_UNKNOWN
} ECHO_state_typedef;

ECHO_state_typedef ECHO_state = ECHO_INIT;
ECHO_state_typedef ECHO_state_last = ECHO_UNKNOWN;

char ECHO_inputstring[1024] = {0};
uint16_t ECHO_inputstring_pos = 0;
uint8_t ECHO_buffer[1024] = {0};
ssize_t ECHO_recv_result = -1;
uint64_t ECHO_transferred = 0;
uint32_t ECHO_start_transfer = 0;


#endif //ETHTEST_STARTECHOTASK_H
