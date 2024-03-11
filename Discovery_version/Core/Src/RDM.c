#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "rdm_define.h"
#include "RDM.h"

uint16_t* Calc_checksum(int frame_size, uint8_t frame[]){
    uint16_t soma = 0;
        for(int i = 0; i < frame_size; i++){     // Cálculo do checksum
            soma = soma + frame[i];
        }
    return soma;
}

uint8_t* Set_frame(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Message_Length, uint8_t Port_ID, uint8_t Message_Count, uint16_t Sub_Device, uint8_t CC, uint16_t PID, uint8_t PDL, uint64_t LB_PD, uint64_t UB_PD){                       // Organiza os dados setados, separando em bytes

    // #Organiza o frame de acordo com os parâmetros passados e no formato do RDM

    static uint8_t* frame;
    frame = (uint8_t*)malloc(Message_Length);

    frame[0] = E120_SC_RDM;                      // Start Code (SC)
    frame[1] = E120_SC_SUB_MESSAGE;              // Sub-Start Code
    frame[2] = Message_Length;                   // Message Length
    frame[3] = (UID_D >> 40) & 0xff;             // Destination UID bit 5
    frame[4] = (UID_D >> 32) & 0xff;             // Destination UID bit 4
    frame[5] = (UID_D >> 24) & 0xff;             // Destination UID bit 3
    frame[6] = (UID_D >> 16) & 0xff;             // Destination UID bit 2
    frame[7] = (UID_D >> 8) & 0xff;              // Destination UID bit 1
    frame[8] = UID_D & 0xff;                     // Destination UID bit 0
    frame[9] = (UID_S >> 40) & 0xff;             // Source UID bit 5
    frame[10] = (UID_S >> 32) & 0xff;            // Source UID bit 4
    frame[11] = (UID_S >> 24) & 0xff;            // Source UID bit 3
    frame[12] = (UID_S >> 16) & 0xff;            // Source UID bit 2
    frame[13] = (UID_S >> 8) & 0xff;             // Source UID bit 1
    frame[14] = UID_S & 0xff;                    // Source UID bit 0
    frame[15] = TN;                              // Transaction Number
    frame[16] = Port_ID;
    frame[17] = Message_Count;
    frame[18] = (Sub_Device >> 8) & 0xff;        // Upper_Sub_Device
    frame[19] = Sub_Device & 0xff;               // Lower_Sub_Device
    frame[20] = CC;                              // Command Class
    frame[21] = (PID >> 8) & 0xff;               // Upper_PID
    frame[22] = PID & 0xff;                      // Lower_PID
    frame[23] = PDL;

    // É possível deixar PD mais fexível, adaptável a qualquer tamanho, utilizando um for que preenche os espaços a partir do PDL

    if (PDL == 12){                              // #Frames que tem 12 bytes de UID (lower and upper bound) Disc_unique_branch
        frame[24] = (LB_PD >> 40) & 0xff;        // Lower Bound UID bit 5
        frame[25] = (LB_PD >> 32) & 0xff;        // Lower Bound UID bit 4
        frame[26] = (LB_PD >> 24) & 0xff;        // Lower Bound UID bit 3
        frame[27] = (LB_PD >> 16) & 0xff;        // Lower Bound UID bit 2
        frame[28] = (LB_PD >> 8) & 0xff;         // Lower Bound UID bit 1
        frame[29] = LB_PD & 0xff;                // Lower Bound UID bit 0
        frame[30] = (UB_PD >> 40) & 0xff;        // Upper Bound UID bit 5
        frame[31] = (UB_PD >> 32) & 0xff;        // Upper Bound UID bit 4
        frame[32] = (UB_PD >> 24) & 0xff;        // Upper Bound UID bit 3
        frame[33] = (UB_PD >> 16) & 0xff;        // Upper Bound UID bit 2
        frame[34] = (UB_PD >> 8) & 0xff;         // Upper Bound UID bit 1
        frame[35] = UB_PD & 0xff;                // Upper Bound UID bit 0

        uint16_t checksum = Calc_checksum(36, frame);
        frame[36] = (checksum >> 8) & 0xff;      // Checksum high
        frame[37] = checksum & 0xff;             // Checksum low

    } else if (PDL == 2) {                       // #Para o Get_parameter_description
        frame[24] =(LB_PD >> 8) & 0xff;
        frame[25] = LB_PD & 0xff;

        uint16_t checksum = Calc_checksum(26, frame);
        frame[26] = (checksum >> 8) & 0xff;
        frame[27] = checksum & 0xff;

    } else if (PDL == 1) {                       // #Para SET_identify_device
        frame[24] = LB_PD & 0xff;

        uint16_t checksum = Calc_checksum(25, frame);
        frame[25] = (checksum >> 8) & 0xff;
        frame[26] = checksum & 0xff;

    } else {                                     // #Para funcões sem PD (disc_mute, disc_un_mute etc.)
        uint16_t checksum = Calc_checksum(24, frame);
        frame[24] = (checksum >> 8) & 0xff;
        frame[25] = checksum & 0xff;
    }
  return (int8_t*)frame;
}

uint8_t* DISC_unique_branch(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint64_t LB_PD, uint64_t UB_PD){

    uint8_t Message_Length = 0x24;// 36 bytes
    uint8_t Message_Count = 0x00;
    uint16_t Sub_Device = 0x0000;
    uint8_t CC = E120_DISCOVERY_COMMAND;
    uint16_t PID = E120_DISC_UNIQUE_BRANCH;
    uint8_t PDL = 0X0c;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;
}

uint8_t* DISC_mute(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID){

    uint8_t Message_Length = 0x18;// 24 bytes
    uint8_t Message_Count = 0x00;
    uint16_t Sub_Device = 0x0000;
    uint8_t CC = E120_DISCOVERY_COMMAND;
    uint16_t PID = E120_DISC_MUTE;
    uint8_t PDL = 0X00;
    uint64_t LB_PD = NULL;
    uint64_t UB_PD = NULL;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;

}

uint8_t* DISC_un_mute(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID){

    uint8_t Message_Length = 0x18;// 24 bytes
    uint8_t Message_Count = 0x00;
    uint16_t Sub_Device = 0x0000;
    uint8_t CC = E120_DISCOVERY_COMMAND;
    uint16_t PID = E120_DISC_UN_MUTE;
    uint8_t PDL = 0X00;
    uint64_t LB_PD = NULL;
    uint64_t UB_PD = NULL;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;

}

uint8_t* GET_supported_parameters(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device){

    uint8_t Message_Length = 0x18;// 24 bytes
    uint8_t Message_Count = 0x00;
    uint8_t CC = E120_GET_COMMAND;
    uint16_t PID = E120_SUPPORTED_PARAMETERS;
    uint8_t PDL = 0X00;
    uint64_t LB_PD = NULL;
    uint64_t UB_PD = NULL;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;

}
uint8_t* GET_parameter_description(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t PID_Requested){

    uint8_t Message_Length = 0x1A;// 26 bytes
    uint8_t Message_Count = 0x00;
    uint16_t Sub_Device = 0x0000;
    uint8_t CC = E120_GET_COMMAND;
    uint16_t PID = E120_PARAMETER_DESCRIPTION;
    uint8_t PDL = 0X02;
    uint64_t LB_PD = PID_Requested;
    uint64_t UB_PD = NULL;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;

}

uint8_t* GET_device_info(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device){

    uint8_t Message_Length = 0x18;// 24 bytes
    uint8_t Message_Count = 0x00;
    uint8_t CC = E120_GET_COMMAND;
    uint16_t PID = E120_DEVICE_INFO;
    uint8_t PDL = 0X00;
    uint64_t LB_PD = NULL;
    uint64_t UB_PD = NULL;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;

}


uint8_t* GET_software_version_label(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device){

    uint8_t Message_Length = 0x18;// 24 bytes
    uint8_t Message_Count = 0x00;
    uint8_t CC = E120_GET_COMMAND;
    uint16_t PID = E120_SOFTWARE_VERSION_LABEL;
    uint8_t PDL = 0X00;
    uint64_t LB_PD = NULL;
    uint64_t UB_PD = NULL;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;

}

uint8_t* GET_dmx_start_address(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device){

    uint8_t Message_Length = 0x18;// 24 bytes
    uint8_t Message_Count = 0x00;
    uint8_t CC = E120_GET_COMMAND;
    uint16_t PID = E120_DMX_START_ADDRESS;
    uint8_t PDL = 0X00;
    uint64_t LB_PD = NULL;
    uint64_t UB_PD = NULL;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;

}

uint8_t* GET_identify_device(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device){

    uint8_t Message_Length = 0x18;// 24 bytes
    uint8_t Message_Count = 0x00;
    uint8_t CC = E120_GET_COMMAND;
    uint16_t PID = E120_IDENTIFY_DEVICE;
    uint8_t PDL = 0X00;
    uint64_t LB_PD = NULL;
    uint64_t UB_PD = NULL;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;

}

uint8_t* SET_dmx_start_address(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device, uint16_t DMX_address){

    uint8_t Message_Length = 0x1A;// 26 bytes
    uint8_t Message_Count = 0x00;
    uint8_t CC = E120_SET_COMMAND;
    uint16_t PID = E120_DMX_START_ADDRESS;
    uint8_t PDL = 0X02;
    uint64_t LB_PD = DMX_address;
    uint64_t UB_PD = NULL;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;

}

uint8_t* SET_identify_device(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device, uint8_t Identify_start_stop){

    uint8_t Message_Length = 0x19;// 25 bytes
    uint8_t Message_Count = 0x00;
    uint8_t CC = E120_SET_COMMAND;
    uint16_t PID = E120_IDENTIFY_DEVICE;
    uint8_t PDL = 0X01;
    uint64_t LB_PD = Identify_start_stop;
    uint64_t UB_PD = NULL;

    uint8_t* data_set = Set_frame(UID_D, UID_S, TN, Message_Length, Port_ID, Message_Count, Sub_Device, CC, PID, PDL, LB_PD, UB_PD);
    return data_set;

}
