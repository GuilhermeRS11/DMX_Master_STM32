#include <SoftwareSerial.h>


int uartPin = 3; // Exemplo: usar pino digital 3 para comunicação UART
SoftwareSerial mySerial(2, uartPin); // RX, TX (pino 2 como RX, pino 3 como TX)

static void DMX_Set_DE_HIGH(void) {digitalWrite(uartPin, HIGH);}
static void DMX_Set_LOW(void)	{digitalWrite(uartPin, LOW);}	 // Seta o Break

uint8_t* receiveBuffer = NULL;
uint8_t dataReceived;
uint16_t receivedIndex = 0;
uint8_t GUI_receiveFinished = 0;
uint8_t GUI_receive = 1;
uint32_t currentTime;



void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial padrão (pino 0 - RX, pino 1 - TX)
  pinMode(uartPin, OUTPUT); // Configura o pino inicialmente como saída para UART
  mySerial.begin(250000); // Inicializa a comunicação serial emulada (pino 2 - RX, pino 3 - TX)
}

void loop() {
  if(GUI_receive == 1){
    /* Recebe dados da GUI */
    if(Serial.available() > 0){
      dataReceived = Serial.read();
      uint8_t* tempBuffer = (uint8_t*)realloc(receiveBuffer, (receivedIndex + 1) * sizeof(uint8_t)); /* Buffer temporario para alocacao dinamica*/
      receiveBuffer = tempBuffer;
      receiveBuffer[receivedIndex++] = dataReceived;
      GUI_receiveFinished = 1; /* Avisa que quando acabar o recebimento de bytes, pode enviar para a luminaria*/

    } else if(GUI_receiveFinished == 1){
      /* Se acabou o recebimento, envia para a luminária e reseta os parametros de recebimento*/
      DMX_send_command(receiveBuffer, receivedIndex);

      if(receiveBuffer[0] == 0xCC){	// Se for um frame DMX, a proxima iteracao sera a espera de um comando vindo da luminaria
        GUI_receive = 0; /* Entra para a secao que espera o recebimento de dados da luminaria e envia para a GUI*/
        //currentTime = __HAL_TIM_GET_COUNTER(&htim2); /* Inicia timer para definir rota de retorno para este modo*/
      }
      GUI_receiveFinished = 0;
      receivedIndex = 0;
      free(receiveBuffer);
      receiveBuffer = NULL;

    }
  } else{
  // Processa resposta  
  }
}

void DMX_send_command(uint8_t* frame, uint8_t size){

	//DMX_Set_DE_HIGH(); // Habilita o barramento DMX para escrita (Necessidade do RS485)
	//delayMicroseconds(10); 	 	 // Espera um tempo antes de iniciar a transmissao

	DMX_Set_DE_HIGH();		 // Seta o MBB
	delayMicroseconds(50);

	DMX_Set_LOW(); 		 // Seta o Break
	delayMicroseconds(250);

	// O Time after break é implementado pela UART, através do idle frame

	for (int i = 0; i < size; i++) {
    mySerial.write(frame[i]); // Envia cada byte individualmente
  }

	//DMX_Set_DE_LOW(); // Desabilita o barramento DMX para escrita (Necessidade do RS485)
}

void testaEnvio(void){
  uint8_t dmx_rdm_data[56] = {0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

    // Envia para a serial debug os dados a serem enviados
    unsigned char viewMessage[30];
    unsigned char viewDMX[30];

    printf(viewMessage, "Frame DMX enviado:\r\n");
    for(int i = 0; i < sizeof(dmx_rdm_data); i++){
      printf("[%d] - 0x%02x\r\n", i, dmx_rdm_data[i]);
    }
    // Faz a transmissão via RS485
    DMX_send_command(dmx_rdm_data, sizeof(dmx_rdm_data));
}
