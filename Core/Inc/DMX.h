#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define TIME_BREAK 250
#define TIME_AFTER_BREAK 20
#define TIME_BEFORE_BREAK 5
#define TIME_WAIT_RDM_RESPONSE 10000 //10ms -> tempo máximo que se espera por uma resposta

void uart_send_break(void);
void uart_send_MAB(void);
void uart_send_MBST(void);
void uart_send_MTBF(void);
void uart_write_byte(uint8_t data, UART_HandleTypeDef *huart);

void send_dmx_frame(uint8_t dmx_data[], int data_size,  UART_HandleTypeDef *huart);
void send_dmx_rdm_frame(uint8_t dmx_rdm_data[], UART_HandleTypeDef *huart);

