#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "rdm_define.h"


uint16_t* Calc_checksum(int frame_size, uint8_t frame[]);

uint8_t* Set_frame(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Message_Length, uint8_t Port_ID, uint8_t Message_Count, uint16_t Sub_Device, uint8_t CC, uint16_t PID, uint8_t PDL, uint64_t LB_PD, uint64_t UB_PD);

uint8_t* DISC_unique_branch(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint64_t LB_PD, uint64_t UB_PD);
uint8_t* DISC_mute(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID);
uint8_t* DISC_un_mute(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID);

uint8_t* GET_supported_parameters(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device);
uint8_t* GET_parameter_description(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t PID_Requested);
uint8_t* GET_device_info(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device);
uint8_t* GET_software_version_label(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device);
uint8_t* GET_dmx_start_address(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device);
uint8_t* GET_identify_device(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device);

uint8_t* SET_dmx_start_address(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device, uint16_t DMX_address);
uint8_t* SET_identify_device(uint64_t UID_D, uint64_t UID_S, uint8_t TN, uint8_t Port_ID, uint16_t Sub_Device, uint8_t Identify_start_stop);

