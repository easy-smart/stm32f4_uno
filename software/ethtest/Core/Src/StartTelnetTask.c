//
// Created by roland on 26.05.2023.
//

#include "StartTelnetTask.h"

int sock=-1, newsock=-1;
struct sockaddr_in server;
struct sockaddr addr;
socklen_t l;


/**
  * @brief  Function implementing the telnetTask thread.
  * @param  argument: Not used
  * @retval None
  */
_Noreturn void StartTelnetTask(void *argument) {
    Debugger_log(DBG, (const char* const) "StartTelnetTask()");
//    TickType_t xLastWakeTime = xTaskGetTickCount();
//    const TickType_t xFrequency = pdMS_TO_TICKS(10);


    /* Infinite loop */
    for (;;) {
        // Wait for the next cycle.
//        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        bool TELNET_state_changed=(TELNET_state != TELNET_state_last);
        TELNET_state_last = TELNET_state;

        switch (TELNET_state) {
            case TELNET_INIT:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET_INIT");
                TELNET_inputstring_pos = 0;
                memset(TELNET_inputstring, 0, sizeof(TELNET_inputstring));
                TELNET_state = TELNET_SOCKET;
                break;

            case TELNET_SOCKET:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET_SOCKET");
                if ((sock = lwip_socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                    // No socket created
                    Debugger_log(DBG, "ERROR: lwip_socket(): %i", sock);
                    osDelay(pdMS_TO_TICKS(1000));
                } else {
                    TELNET_state = TELNET_BIND;
                }

                break;

            case TELNET_BIND:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET_BIND");
                server.sin_family = AF_INET;
                server.sin_addr.s_addr = INADDR_ANY;
                server.sin_port = htons(TELNET_port);
                if (lwip_bind(sock, (struct sockaddr *)&server, sizeof(server))) {
                    Debugger_log(DBG, "ERROR: lwip_bind()");
                    osDelay(pdMS_TO_TICKS(1000));
                } else {
                    TELNET_state = TELNET_LISTEN;
                }
                break;


            case TELNET_LISTEN:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET_LISTEN");
                if(lwip_listen(sock, 5)) {
                    Debugger_log(DBG, "ERROR: lwip_listen()");
                    osDelay(pdMS_TO_TICKS(1000));
                } else {
                    TELNET_state = TELNET_IDLE;
                }
                break;


            case TELNET_IDLE:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET_IDLE");

                if((newsock=lwip_accept(sock, &addr, &l)) >= 0) {
                    TELNET_state = TELNET_READ;
                }
                break;

            case TELNET_READ:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET_READ");

                TELNET_recv_result=lwip_read(newsock, TELNET_buffer, sizeof(TELNET_buffer));

                if (TELNET_recv_result < 0) {
                    Debugger_log(DBG, "ERROR lwip_read(): %li", errno);
                    if(errno==ENOTCONN) {
                        TELNET_state = TELNET_CLOSE;
                    } else {
                        osDelay(pdMS_TO_TICKS(1000));
                    }
                } else if (TELNET_recv_result == 1) {
                    if (TELNET_inputstring_pos < sizeof(TELNET_inputstring)) {
                        TELNET_inputstring[TELNET_inputstring_pos++] = TELNET_buffer[0];
                        if (TELNET_buffer[0] == '\n') TELNET_state = TELNET_RECEIVED;
                    } else {
                        Debugger_log(DBG, "ERROR: buffer overflow");
                        TELNET_state = TELNET_RECEIVED;
                    }
                } else if (TELNET_recv_result > 1) {
                    Debugger_log(DBG, "ERROR: NOT SUPPORTED");
                    TELNET_state = TELNET_ERROR;
                }

                break;

            case TELNET_RECEIVED:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET_RECEIVED");
                Debugger_log(DBG, "received: %s", TELNET_inputstring);

                if(TELNET_inputstring[0] == '0') {
                    HAL_GPIO_WritePin(J110_8_LED1_GRN_GPIO_Port,J110_8_LED1_GRN_Pin,GPIO_PIN_SET);
                    HAL_GPIO_WritePin(J110_7_LED2_ORG_GPIO_Port,J110_7_LED2_ORG_Pin,GPIO_PIN_SET);
                    HAL_GPIO_WritePin(J110_6_LED3_RED_GPIO_Port,J110_6_LED3_RED_Pin,GPIO_PIN_SET);
                    HAL_GPIO_WritePin(J110_5_LED4_BLU_GPIO_Port,J110_5_LED4_BLU_Pin,GPIO_PIN_SET);
                }

                if(TELNET_inputstring[0] == 'F') {
                    HAL_GPIO_WritePin(J110_8_LED1_GRN_GPIO_Port,J110_8_LED1_GRN_Pin,GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(J110_7_LED2_ORG_GPIO_Port,J110_7_LED2_ORG_Pin,GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(J110_6_LED3_RED_GPIO_Port,J110_6_LED3_RED_Pin,GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(J110_5_LED4_BLU_GPIO_Port,J110_5_LED4_BLU_Pin,GPIO_PIN_RESET);
                }

                if(TELNET_inputstring[0] == '1') {
                    HAL_GPIO_TogglePin(J110_8_LED1_GRN_GPIO_Port, J110_8_LED1_GRN_Pin);
                }
                if(TELNET_inputstring[0] == '2') {
                    HAL_GPIO_TogglePin(J110_7_LED2_ORG_GPIO_Port, J110_7_LED2_ORG_Pin);
                }
                if(TELNET_inputstring[0] == '3') {
                    HAL_GPIO_TogglePin(J110_6_LED3_RED_GPIO_Port, J110_6_LED3_RED_Pin);
                }
                if(TELNET_inputstring[0] == '4') {
                    HAL_GPIO_TogglePin(J110_5_LED4_BLU_GPIO_Port, J110_5_LED4_BLU_Pin);
                }


                lwip_write(newsock, "ok\r\n", 4);
                TELNET_inputstring_pos = 0;
                memset(TELNET_inputstring, 0, sizeof(TELNET_inputstring));
                TELNET_state = TELNET_READ;
                break;

            case TELNET_CLOSE:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET_CLOSE");
                lwip_close(newsock);
                newsock=-1;
                TELNET_state = TELNET_IDLE;
                break;

            case TELNET_DISABLED:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET_DISABLED");
                break;

            case TELNET_ERROR:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET_ERROR");
                break;

            default:
                if (TELNET_state_changed) Debugger_log(DBG, "TELNET default");
        }



        osDelay(1);
    }
}
