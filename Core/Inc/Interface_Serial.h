#include <windows.h>
#include <locale.h> // Incluir isso no início do arquivo para set de caracteres em portugues BR
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>

void tic();
double toc();
void toc_escreve();
void printHexVetor(unsigned char * v,int numCharValidos);
void printASCIIVetor(unsigned char * v,int numCharValidos);
int serial_interface(uint8_t data[], int data_size);
