#ifndef __MODULE_H__
#define __MODULE_H__

// Includes:

    #include "module/module_api.h"
    #include "key_vector.h"
    #include "module/slave_protocol_handler.h"

// Macros:

    #define I2C_ADDRESS_MODULE_FIRMWARE I2C_ADDRESS_RIGHT_MODULE_FIRMWARE
    #define I2C_ADDRESS_MODULE_BOOTLOADER I2C_ADDRESS_RIGHT_MODULE_BOOTLOADER

    #define MODULE_PROTOCOL_VERSION 1
    #define MODULE_ID ModuleId_TrackballRight
    #define MODULE_KEY_COUNT KEYBOARD_VECTOR_ITEMS_NUM
    #define MODULE_POINTER_COUNT 1

    #define TEST_LED_GPIO  GPIOB
    #define TEST_LED_PORT  PORTB
    #define TEST_LED_CLOCK kCLOCK_PortB
    #define TEST_LED_PIN   2

    #define KEY_ARRAY_TYPE KEY_ARRAY_TYPE_VECTOR
    #define KEYBOARD_VECTOR_ITEMS_NUM 2

// Variables:

    extern key_vector_t keyVector;
    extern pointer_delta_t PointerDelta;

    extern const bool PG_enabled;
    extern uint8_t PG_regionBuffer[PG_REGION_SIZE];
    extern uint8_t PG_regionIdx;
    extern uint8_t PG_inRegionIdx;

// Functions:

    void Module_Init(void);
    void Module_Loop(void);

#endif
