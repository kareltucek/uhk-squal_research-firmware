#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_spi.h"
#include "module.h"
#include "slave_protocol.h"

#define TRACKBALL_SHTDWN_PORT PORTA
#define TRACKBALL_SHTDWN_GPIO GPIOA
#define TRACKBALL_SHTDWN_IRQ PORTA_IRQn
#define TRACKBALL_SHTDWN_CLOCK kCLOCK_PortA
#define TRACKBALL_SHTDWN_PIN 4

#define TRACKBALL_NCS_PORT PORTB
#define TRACKBALL_NCS_GPIO GPIOB
#define TRACKBALL_NCS_IRQ PORTB_IRQn
#define TRACKBALL_NCS_CLOCK kCLOCK_PortB
#define TRACKBALL_NCS_PIN 1

#define TRACKBALL_MOSI_PORT PORTA
#define TRACKBALL_MOSI_GPIO GPIOA
#define TRACKBALL_MOSI_IRQ PORTA_IRQn
#define TRACKBALL_MOSI_CLOCK kCLOCK_PortA
#define TRACKBALL_MOSI_PIN 7

#define TRACKBALL_MISO_PORT PORTA
#define TRACKBALL_MISO_GPIO GPIOA
#define TRACKBALL_MISO_IRQ PORTA_IRQn
#define TRACKBALL_MISO_CLOCK kCLOCK_PortA
#define TRACKBALL_MISO_PIN 6

#define TRACKBALL_SCK_PORT PORTB
#define TRACKBALL_SCK_GPIO GPIOB
#define TRACKBALL_SCK_IRQ PORTB_IRQn
#define TRACKBALL_SCK_CLOCK kCLOCK_PortB
#define TRACKBALL_SCK_PIN 0

#define TRACKBALL_SPI_MASTER SPI0
#define TRACKBALL_SPI_MASTER_SOURCE_CLOCK kCLOCK_BusClk

pointer_delta_t PointerDelta;

key_vector_t keyVector = {
    .itemNum = KEYBOARD_VECTOR_ITEMS_NUM,
    .items = (key_vector_pin_t[]) {
        {PORTA, GPIOA, kCLOCK_PortA, 3}, // left button
        {PORTA, GPIOA, kCLOCK_PortA, 12}, // right button
    },
};

#define BUFFER_SIZE 2
#define MOTION_BIT (1<<7)
#define PIXRDY_BIT (1<<6)
#define PIXFST_BIT (1<<5)

typedef enum {
    ModulePhase_SetResolution,
    ModulePhase_Initialized,
    ModulePhase_ProcessMotion,
    ModulePhase_ProcessDeltaY,
    ModulePhase_ProcessDeltaX,
    ModulePhase_ProcessSqual,
    ModulePhase_ProcessShutterLower,
    ModulePhase_ProcessShutterUpper,
	ModulePhase_ProcessPixgrab
} module_phase_t;

module_phase_t modulePhase = ModulePhase_SetResolution;

uint8_t txBufferPowerUpReset[] = {0xba, 0x5a};
uint8_t txSetResolution[] = {0x91, 0b10000000};
uint8_t txBufferGetMotion[] = {0x02, 0x00};
uint8_t txBufferGetDeltaY[] = {0x03, 0x00};
uint8_t txBufferGetDeltaX[] = {0x04, 0x00};
uint8_t txBufferGetSQUAL[] = {0x05, 0x00};
uint8_t txBufferGetShutterUpper[] = {0x06, 0x00};
uint8_t txBufferGetShutterLower[] = {0x07, 0x00};
uint8_t txBufferGetPixel[] = {0x0b, 0x00};
uint8_t txBufferInitPixgrab[] = {0x0b, 0x01};



uint8_t rxBuffer[BUFFER_SIZE];
spi_master_handle_t handle;
spi_transfer_t xfer = {0};

bool takeSqual = true;

void tx(uint8_t *txBuff)
{
    GPIO_WritePinOutput(TRACKBALL_NCS_GPIO, TRACKBALL_NCS_PIN, 1);
    GPIO_WritePinOutput(TRACKBALL_NCS_GPIO, TRACKBALL_NCS_PIN, 0);
    xfer.txData = txBuff;
    SPI_MasterTransferNonBlocking(TRACKBALL_SPI_MASTER, &handle, &xfer);
}

static uint8_t motion = 0;

static void schedulePixgrabMaybe(bool allowPixgrab) {

    if(allowPixgrab && PG_enabled && (motion & PIXRDY_BIT) && PG_inRegionIdx < PG_REGION_SIZE) {
        tx(txBufferGetPixel);
        modulePhase = ModulePhase_ProcessPixgrab;
    } else {
        tx(txBufferGetMotion);
        modulePhase = ModulePhase_ProcessMotion;
    }
}

static void processPixgrab() {
    if(motion & PIXFST_BIT) {
        PG_inRegionIdx = 0;
        PG_regionIdx = 0;
    }
    PG_regionBuffer[PG_inRegionIdx] = (uint8_t)rxBuffer[1];
    PG_inRegionIdx++;
}

void trackballUpdate(SPI_Type *base, spi_master_handle_t *masterHandle, status_t status, void *userData)
{
    switch (modulePhase) {
        case ModulePhase_SetResolution:
            tx(txSetResolution);
            modulePhase = ModulePhase_Initialized;
            break;
        case ModulePhase_Initialized:
            tx(txBufferGetMotion);
            modulePhase = ModulePhase_ProcessMotion;
            break;
        case ModulePhase_ProcessMotion: ;
            motion = rxBuffer[1];
            bool isMoved = motion & MOTION_BIT;
            if (isMoved) {
                tx(txBufferGetDeltaY);
                modulePhase = ModulePhase_ProcessDeltaY;
            } else {
                schedulePixgrabMaybe(true);
            }
            break;
        case ModulePhase_ProcessDeltaY: ;
            int8_t deltaY = (int8_t)rxBuffer[1];
            PointerDelta.x += deltaY; // This is correct given the sensor orientation.
            PointerDelta.maxY = deltaY > PointerDelta.maxY ? deltaY : PointerDelta.maxY;
            tx(txBufferGetDeltaX);
            modulePhase = ModulePhase_ProcessDeltaX;
            break;
        case ModulePhase_ProcessDeltaX: ;
            int8_t deltaX = (int8_t)rxBuffer[1];
            PointerDelta.y += deltaX; // This is correct given the sensor orientation.
            if(takeSqual) {
                tx(txBufferGetSQUAL);
                modulePhase = ModulePhase_ProcessSqual;
            } else {
                schedulePixgrabMaybe(false);
            }
            break;
        case ModulePhase_ProcessSqual: ;
            uint8_t squal = (uint8_t)rxBuffer[1];
            PointerDelta.squal = squal;
            tx(txBufferGetShutterUpper);
            modulePhase = ModulePhase_ProcessShutterUpper;
            break;
        case ModulePhase_ProcessShutterUpper: ;
			uint16_t shutterUpper = (uint8_t)rxBuffer[1];
			PointerDelta.shutter = (PointerDelta.shutter & 0x00FF) + (shutterUpper * 256);
            tx(txBufferGetShutterLower);
            modulePhase = ModulePhase_ProcessShutterLower;
            break;
        case ModulePhase_ProcessShutterLower: ;
			uint16_t shutterLower = (uint8_t)rxBuffer[1];
			PointerDelta.shutter = (PointerDelta.shutter & 0xFF00) + shutterLower;
            schedulePixgrabMaybe(true);
            break;
        case ModulePhase_ProcessPixgrab: ;
            processPixgrab();
            schedulePixgrabMaybe(false);
            break;
    }
}

void Trackball_Init(void)
{
    CLOCK_EnableClock(TRACKBALL_SHTDWN_CLOCK);
    PORT_SetPinMux(TRACKBALL_SHTDWN_PORT, TRACKBALL_SHTDWN_PIN, kPORT_MuxAsGpio);
    GPIO_WritePinOutput(TRACKBALL_SHTDWN_GPIO, TRACKBALL_SHTDWN_PIN, 0);

    CLOCK_EnableClock(TRACKBALL_NCS_CLOCK);
    PORT_SetPinMux(TRACKBALL_NCS_PORT, TRACKBALL_NCS_PIN, kPORT_MuxAsGpio);

    CLOCK_EnableClock(TRACKBALL_MOSI_CLOCK);
    PORT_SetPinMux(TRACKBALL_MOSI_PORT, TRACKBALL_MOSI_PIN, kPORT_MuxAlt3);

    CLOCK_EnableClock(TRACKBALL_MISO_CLOCK);
    PORT_SetPinMux(TRACKBALL_MISO_PORT, TRACKBALL_MISO_PIN, kPORT_MuxAlt3);

    CLOCK_EnableClock(TRACKBALL_SCK_CLOCK);
    PORT_SetPinMux(TRACKBALL_SCK_PORT, TRACKBALL_SCK_PIN, kPORT_MuxAlt3);

    uint32_t srcFreq = 0;
    spi_master_config_t userConfig;
    SPI_MasterGetDefaultConfig(&userConfig);
    userConfig.polarity = kSPI_ClockPolarityActiveLow;
    userConfig.phase = kSPI_ClockPhaseSecondEdge;
    userConfig.baudRate_Bps = 100000U;
    srcFreq = CLOCK_GetFreq(TRACKBALL_SPI_MASTER_SOURCE_CLOCK);
    SPI_MasterInit(TRACKBALL_SPI_MASTER, &userConfig, srcFreq);
    SPI_MasterTransferCreateHandle(TRACKBALL_SPI_MASTER, &handle, trackballUpdate, NULL);
    xfer.rxData = rxBuffer;
    xfer.dataSize = BUFFER_SIZE;
    tx(txBufferPowerUpReset);
}

void Module_Init(void)
{
    KeyVector_Init(&keyVector);
    Trackball_Init();
}

void Module_Loop(void)
{
}
