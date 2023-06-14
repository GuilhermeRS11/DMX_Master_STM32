#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


void uart_send_break(void);
void uart_send_MAB(void);
void uart_send_MBST(void);
void uart_send_MTBF(void);
void uart_write_byte(uint8_t data, UART_HandleTypeDef *huart);

void send_dmx_frame(uint8_t dmx_data[], int data_size,  UART_HandleTypeDef *huart);
void send_dmx_rdm_frame(uint8_t dmx_rdm_data[], UART_HandleTypeDef *huart);
