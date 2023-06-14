#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f0xx_hal.h"
#include "DMX.h"

#define BOUND_RATE 250000
#define NUM_CHANNELS 40
//#define DEBUG
#define TIMEOUT 100

void uart_send_break(){
    // Envia um low level avisar que uma nova mensagem DMX ir� come�ar
    // Deve ser entre 12us e 88us no transmissor e entre 176us e 352us no receptor
    //return 0;
}

void uart_send_MAB(){
    //  MAB helps to ensure the Break is an actual start sequence, and also provides sufficient time
    //for even slow receivers to reset after the previous frame
    // Deve ser entre 12us e 88us no transmissor e entre 8us e 88us no receptor
    //return 0;
}

void uart_send_MBST(){
    // Mark-Between-Slot Time
    // Deve ser entre 0 e 2ms para o transmissor e entre 0 e 2.1ms para o receptor
    //return 0;
}

void uart_send_MTBF(){
    // Mark Time Between Frames
    // Deve ser setado em 1 e pode ser acima de 1s
    //return 0;
}

void uart_write_byte(uint8_t data, UART_HandleTypeDef *huart){
    // Envia:
    // Start bit
    // Byte dos dados
    // 2 stop bit

	HAL_UART_Transmit(huart, data, 1, TIMEOUT);

	#ifdef DEBUG
        printf("\ndata = 0x%02x", data);
    #endif // DEBUG
    //return 0;
}

void send_dmx_frame(uint8_t dmx_data[], int data_size, UART_HandleTypeDef *huart) {
    // Implementa apenas o DMX

    uart_send_break();                              // Manda um break para avisar que uma nova mensagem DMX ir� come�ar e logo depois o mark after break
    uart_send_MAB();

    uart_write_byte(0x00, huart);                          // Envia o start_byte
    uart_send_MBST();

    // Envia os slots, sempre seguidos do MTBS
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if(i < data_size){                          // Quando os dados a serem transmitidos s�o dos do RDM
            uart_write_byte(dmx_data[i], huart);
            uart_send_MBST();
        } else{                                     // Quando os dados j� n�o s�o mais os do RDM
            uart_write_byte(0x00, huart);
            uart_send_MBST();
        }

  }
    // Final do frame DMX. Espera para a nova transmiss�o
    uart_send_MTBF();
}

/*
void send_dmx_rdm_frame(uint8_t dmx_rdm_data[]){

    // Envia o RDM atrav�s do DMX

    uint8_t dmx_rdm_data_size = dmx_rdm_data[2] + 2;

    uart_send_break();                              // Manda um break para avisar que uma nova mensagem DMX ir� come�ar e logo depois o mark after break
    uart_send_MAB();

    // Envia cada byte do frame RDM, colocando um MTBS depois de cada byte
    for (int i = 0; i < NUM_CHANNELS; i++) {

        if(i < dmx_rdm_data_size){                  // Quando os dados a serem transmitidos s�o dos do RDM
            uart_write_byte(dmx_rdm_data[i]);
            uart_send_MBST();
        } else{                                     // Quando os dados j� n�o s�o mais os do RDM
            uart_write_byte(0x00);
            uart_send_MBST();
        }
    }
    // Final do frame DMX. Espera para a nova transmiss�o
    uart_send_MTBF();
}*/
