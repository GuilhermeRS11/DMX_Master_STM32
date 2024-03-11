#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "rdm_define.h"

/*uint32_t catch_ID(frame){

    return
}*/


// Adicionar os frames que faltam, ler a respeito deles e começar os Get e Set

typedef union{                          //Defines the RDM_Data class
    unsigned char value[24];
    struct{
        union{
            uint16_t CS;                                                //value[0~1]
            struct{
                unsigned CSL:8;         //ChechSum Low
                unsigned CSH:8;         //ChechSum High
            };
        };

        unsigned PDL:8;                 //Parameter Data Length         //value[2]
        uint16_t PID;                   //Parameter ID                  //value[3~4]

        unsigned CC:8;                  //Command Class                 //value[5]
        uint16_t subDevice;             //sub device                    //value[6~7]
        unsigned message:8;             //message count                 //value[8]
        unsigned PORT:8;                //Port ID / Response Type       //value[9]
        unsigned TN:8;                  //Transaction Number            //value[10]

        struct{
            uint32_t ID;
            uint16_t M;                 //Manufacture
        }SUID;                          //Source UID                    //value[11~16]

        struct{
            uint32_t ID;
            uint16_t M;                  //Manufacture
        }DUID;                           // Destination UID             //value[17~22]

        unsigned ML:8;                   //MessageLength                //value[23]
    };
}RDM_Data;

typedef struct{

    unsigned char a;
    unsigned char b;
    unsigned char c;

}Frame_data;

