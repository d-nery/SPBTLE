#ifndef _BLE_H_
#define _BLE_H_

#include "stm32f3xx_hal.h"
#include "ble_status.h"

#define SYSCLK_FREQ 72000000                                 // Change this
#define SYSCLK_FREQ_SLEEP 32000

/**
 * SPI and other SPBTLE pins definitions
 * Get them from HAL
 */
// SPI Instance
#define BNRG_SPI_HANDLER            hspi1                     // Change this
#define BNRG_SPI_INSTANCE           SPI1                      // Change this
#define BNRG_SPI_CLK_ENABLE()       __SPI1_CLK_ENABLE()       // Change this

// SPI Configuration
#define BNRG_SPI_MODE               SPI_MODE_MASTER
#define BNRG_SPI_DIRECTION          SPI_DIRECTION_2LINES
#define BNRG_SPI_DATASIZE           SPI_DATASIZE_8BIT
#define BNRG_SPI_CLKPOLARITY        SPI_POLARITY_LOW
#define BNRG_SPI_CLKPHASE           SPI_PHASE_1EDGE
#define BNRG_SPI_NSS                SPI_NSS_SOFT
#define BNRG_SPI_FIRSTBIT           SPI_FIRSTBIT_MSB
#define BNRG_SPI_TIMODE             SPI_TIMODE_DISABLED
#define BNRG_SPI_CRCPOLYNOMIAL      7
#define BNRG_SPI_BAUDRATEPRESCALER  SPI_BAUDRATEPRESCALER_8
#define BNRG_SPI_CRCCALCULATION     SPI_CRCCALCULATION_DISABLED

// SPI Reset Pin: PA.8
#define BNRG_SPI_RESET_PORT         GPIOA                    // Change this
#define BNRG_SPI_RESET_PIN          GPIO_PIN_8               // Change this
#define BNRG_SPI_RESET_CLK_ENABLE() __GPIOA_CLK_ENABLE()     // Change this
#define BNRG_SPI_RESET_MODE         GPIO_MODE_OUTPUT_PP
#define BNRG_SPI_RESET_PULL         GPIO_PULLUP
#define BNRG_SPI_RESET_SPEED        GPIO_SPEED_LOW
#define BNRG_SPI_RESET_ALTERNATE    0

// SCLK: PB.3
#define BNRG_SPI_SCLK_PORT          GPIOA                    // Change this
#define BNRG_SPI_SCLK_PIN           GPIO_PIN_5               // Change this
#define BNRG_SPI_SCLK_CLK_ENABLE()  __GPIOA_CLK_ENABLE()     // Change this
#define BNRG_SPI_SCLK_MODE          GPIO_MODE_AF_PP
#define BNRG_SPI_SCLK_PULL          GPIO_PULLDOWN
#define BNRG_SPI_SCLK_SPEED         GPIO_SPEED_HIGH
#define BNRG_SPI_SCLK_ALTERNATE     GPIO_AF5_SPI1

// MISO (Master Input Slave Output): PA.6
#define BNRG_SPI_MISO_PORT          GPIOA                    // Change this
#define BNRG_SPI_MISO_PIN           GPIO_PIN_6               // Change this
#define BNRG_SPI_MISO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()     // Change this
#define BNRG_SPI_MISO_MODE          GPIO_MODE_AF_PP
#define BNRG_SPI_MISO_PULL          GPIO_PULLUP
#define BNRG_SPI_MISO_SPEED         GPIO_SPEED_HIGH
#define BNRG_SPI_MISO_ALTERNATE     GPIO_AF5_SPI1

// MOSI (Master Output Slave Input): PA.7
#define BNRG_SPI_MOSI_PORT          GPIOA                    // Change this
#define BNRG_SPI_MOSI_PIN           GPIO_PIN_7               // Change this
#define BNRG_SPI_MOSI_CLK_ENABLE()  __GPIOA_CLK_ENABLE()     // Change this
#define BNRG_SPI_MOSI_MODE          GPIO_MODE_AF_PP
#define BNRG_SPI_MOSI_PULL          GPIO_NOPULL
#define BNRG_SPI_MOSI_SPEED         GPIO_SPEED_HIGH
#define BNRG_SPI_MOSI_ALTERNATE     GPIO_AF5_SPI1

// NSS/CSN/CS: PA.1
#define BNRG_SPI_CS_PORT            GPIOA                    // Change this
#define BNRG_SPI_CS_PIN             GPIO_PIN_1               // Change this
#define BNRG_SPI_CS_CLK_ENABLE()    __GPIOA_CLK_ENABLE()     // Change this
#define BNRG_SPI_CS_MODE            GPIO_MODE_OUTPUT_PP
#define BNRG_SPI_CS_PULL            GPIO_PULLUP
#define BNRG_SPI_CS_SPEED           GPIO_SPEED_HIGH
#define BNRG_SPI_CS_ALTERNATE       0

// IRQ: PA.0
#define BNRG_SPI_IRQ_PORT           GPIOA                    // Change this
#define BNRG_SPI_IRQ_PIN            GPIO_PIN_0               // Change this
#define BNRG_SPI_IRQ_CLK_ENABLE()   __GPIOA_CLK_ENABLE()     // Change this
#define BNRG_SPI_IRQ_MODE           GPIO_MODE_IT_RISING
#define BNRG_SPI_IRQ_PULL           GPIO_NOPULL
#define BNRG_SPI_IRQ_SPEED          GPIO_SPEED_FREQ_HIGH
#define BNRG_SPI_IRQ_ALTERNATE      0

// EXTI External Interrupt for SPI
// NOTE: if you change the IRQ pin remember to implement a corresponding handler
// function like EXTI0_IRQHandler() in the user project
#define BNRG_SPI_EXTI_IRQn          EXTI0_IRQn               // Change this
#define BNRG_SPI_EXTI_IRQHandler    EXTI0_IRQHandler         // Change this
#define BNRG_SPI_EXTI_PIN           BNRG_SPI_IRQ_PIN
#define BNRG_SPI_EXTI_PORT          BNRG_SPI_IRQ_PORT

uint8_t bt_recv;

/**
 * Public function prototypes
 */
tBleStatus BlueNRG_Init();
void BlueNRG_RST(void);
uint8_t BlueNRG_DataPresent(void);
int32_t BlueNRG_SPI_Read_All(uint8_t *buffer, uint8_t buff_size);
int32_t BlueNRG_SPI_Write(uint8_t* data1, uint8_t* data2, uint8_t Nb_bytes1, uint8_t Nb_bytes2);

void Hal_Write_Serial(const void* data1, const void* data2, int32_t n_bytes1, int32_t n_bytes2);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void Enable_SPI_IRQ(void);
void Disable_SPI_IRQ(void);
void Clear_SPI_IRQ(void);
void Clear_SPI_EXTI_Flag(void);

tBleStatus Add_LED_Service(void);
tBleStatus Add_My_Service(void);
void User_Process();
void setConnectable();

tBleStatus Update_Charac(uint8_t value);


#endif
