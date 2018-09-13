/**
 * @file main.c
 *
 * @brief Main function
 */

#include "bluenrg_interface.h"
#include "debug.h"
#include "hci.h"
#include "mcu.h"
#include "spi.h"

/*****************************************
 * Private Constant Definitions
 *****************************************/

/*****************************************
 * Main Function
 *****************************************/

int main(void) {
    mcu_init();

    PRINTF("Iniciando...\n");

    mcu_sleep(1000);

    MX_SPI1_Init();

    PRINTF("SP1 Initialized!\n");

    BlueNRG_Init();
    // User_Process();

    // Add_LED_Service();

    for (;;) {
        mcu_sleep(1000);
        // HCI_Process();
    }
}

int _write(int file, char* ptr, int len) {
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++)
        ITM_SendChar(*ptr++);

    return len;
}
