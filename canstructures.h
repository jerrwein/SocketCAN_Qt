#ifndef CANSTRUCTURES_H
#define CANSTRUCTURES_H

#include <stdint.h>

typedef struct
{
    struct can_frame 	CanFrame;
    unsigned long long	usec_delta;
    unsigned long		tv_usec;
} CAN_DATA;

typedef struct SEG_TRANSFER_INFO
{
//	SEG_TRANSFER_PHASE	TransferPhase;
    uint16_t		primary_index;
    uint8_t			sub_index;
    uint8_t			u8_toggle_state;
    uint8_t			data[100];
    uint32_t		u32_BytesExpected;
    uint32_t		u32_BytesTaken;
} SEG_TRANSFER_INFO;

#endif // CANSTRUCTURES_H

