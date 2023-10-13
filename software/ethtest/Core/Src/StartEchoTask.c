//
// Created by roland on 13.10.2023.
//

#include "StartEchoTask.h"

/**
  * @brief  Function implementing the telnetTask thread.
  * @param  argument: Not used
  * @retval None
  */
_Noreturn void StartEchoTask(void *argument) {
    Debugger_log(DBG, (const char *const) "StartEchoTask()");
    /* Infinite loop */

    int sock=-1, newsock=-1;
    struct sockaddr_in server;
    struct sockaddr addr;
    socklen_t l;

    for (;;) {
        bool ECHO_state_changed=(ECHO_state != ECHO_state_last);
        ECHO_state_last = ECHO_state;

        switch (ECHO_state) {
            case ECHO_INIT:
                if (ECHO_state_changed) Debugger_log(DBG, "ECHO_INIT");
                ECHO_inputstring_pos = 0;
                memset(ECHO_inputstring, 0, sizeof(ECHO_inputstring));
                ECHO_state = ECHO_SOCKET;
                break;

            case ECHO_SOCKET:
                if (ECHO_state_changed) Debugger_log(DBG, "ECHO_SOCKET");
                if ((sock = lwip_socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                    // No socket created
                    Debugger_log(DBG, "ERROR: lwip_socket(): %li", errno);
                    osDelay(pdMS_TO_TICKS(1000));
                } else {
                    ECHO_state = ECHO_BIND;
                }
                break;

            case ECHO_BIND:
                if (ECHO_state_changed) Debugger_log(DBG, "ECHO_BIND");
                server.sin_family = AF_INET;
                server.sin_addr.s_addr = INADDR_ANY;
                server.sin_port = htons(ECHO_port);
                if (lwip_bind(sock, (struct sockaddr *)&server, sizeof(server))) {
                    Debugger_log(DBG, "ERROR: lwip_bind(): %li", errno);
                    osDelay(pdMS_TO_TICKS(1000));
                } else {
                    ECHO_state = ECHO_LISTEN;
                }
                break;

            case ECHO_LISTEN:
                if (ECHO_state_changed) Debugger_log(DBG, "ECHO_LISTEN");
                if(lwip_listen(sock, 5)) {
                    Debugger_log(DBG, "ERROR: lwip_listen(): %li", errno);
                    osDelay(pdMS_TO_TICKS(1000));
                } else {
                    ECHO_state = ECHO_IDLE;
                }
                break;

            case ECHO_IDLE:
                if (ECHO_state_changed) Debugger_log(DBG, "ECHO_IDLE");

                if((newsock=lwip_accept(sock, &addr, &l)) >= 0) {
                    ECHO_transferred = 0;
                    ECHO_start_transfer = HAL_GetTick();
                    ECHO_state = ECHO_READ;
                }
                break;

            case ECHO_READ:
                if (ECHO_state_changed) {
                    memset(ECHO_inputstring, 0, sizeof(ECHO_inputstring));
                    lwip_inet_ntop(addr.sa_family, &(((struct sockaddr_in *) (&addr))->sin_addr), ECHO_inputstring,
                                   sizeof(ECHO_inputstring) - 1);
//                if(addr.sa_family == AF_INET) {
//                    lwip_inet_ntop(addr.sa_family, &(((struct sockaddr_in *)(&addr))->sin_addr), ECHO_inputstring, sizeof(ECHO_inputstring)-1);
//                } else if(addr.sa_family == AF_INET6) {
//                    lwip_inet_ntop(addr.sa_family, &(((struct sockaddr_in *)(&addr))->sin_addr), ECHO_inputstring, sizeof(ECHO_inputstring)-1);
//                }

                    Debugger_log(DBG, "ECHO_READ (%s)", ECHO_inputstring);
                }

                memset(ECHO_buffer, 0, sizeof(ECHO_buffer));
                ECHO_recv_result=lwip_read(newsock, ECHO_buffer, sizeof(ECHO_buffer));

                if (ECHO_recv_result < 0) {
                    Debugger_log(DBG, "ERROR lwip_read(): %li", errno);
                    if(errno==ENOTCONN) {
                        ECHO_state = ECHO_CLOSE;
                    } else {
                        osDelay(pdMS_TO_TICKS(1000));
                    }
                } else if (ECHO_recv_result >= 1) {
                    ECHO_transferred+=ECHO_recv_result;
                    Debugger_log(DBG, "transferred: %li B (%lli B) in %li ms = %lli kBit/s", ECHO_recv_result, ECHO_transferred, (HAL_GetTick() - ECHO_start_transfer), (8000*ECHO_transferred / ((uint64_t)(HAL_GetTick() - ECHO_start_transfer)*1024)));
//                    lwip_write(newsock, ECHO_buffer, ECHO_recv_result);
                }
                break;

            case ECHO_RECEIVED:
                if (ECHO_state_changed) Debugger_log(DBG, "ECHO_RECEIVED: NOT IMPLEMENTED");
                ECHO_state = ECHO_ERROR;
                break;

            case ECHO_CLOSE:
                if (ECHO_state_changed) Debugger_log(DBG, "ECHO_CLOSE");
                lwip_close(newsock);
                newsock=-1;
                ECHO_state = ECHO_IDLE;
                break;

            case ECHO_DISABLED:
                if (ECHO_state_changed) Debugger_log(DBG, "ECHO_DISABLED");
                break;

            case ECHO_ERROR:
                if (ECHO_state_changed) Debugger_log(DBG, "ECHO_ERROR");
                break;

            default:
                if (ECHO_state_changed) Debugger_log(DBG, "ECHO default");
        }

        osDelay(1);
    }
}

