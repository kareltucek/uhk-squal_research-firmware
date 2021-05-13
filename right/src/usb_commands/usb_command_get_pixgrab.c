#include "fsl_common.h"
#include "usb_commands/usb_command_get_device_property.h"
#include "usb_protocol_handler.h"
#include "eeprom.h"
#include "versions.h"
#include "slave_drivers/kboot_driver.h"
#include "slave_drivers/uhk_module_driver.h"
#include "i2c.h"
#include "init_peripherals.h"
#include "fsl_i2c.h"
#include "timer.h"

void UsbCommand_GetPixgrab(void)
{
    uint8_t regionIdx = GetUsbRxBufferUint8(1);

    memcpy(GenericHidOutBuffer, PG_buffer + regionIdx*PG_REGION_SIZE, PG_REGION_SIZE);
}
