#ifndef __SLAVE_PROTOCOL_HANDLER_H__
#define __SLAVE_PROTOCOL_HANDLER_H__

// Includes:

    #include "fsl_port.h"
    #include "crc16.h"
    #include "slave_protocol.h"

// Variables:

    extern i2c_message_t RxMessage;
    extern i2c_message_t TxMessage;


    extern const bool PG_enabled;
    extern uint8_t PG_regionBuffer[PG_REGION_SIZE];
    extern uint8_t PG_regionIdx;
    extern uint8_t PG_inRegionIdx;

// Functions:

    void SlaveRxHandler(void);
    void SlaveTxHandler(void);

#endif
