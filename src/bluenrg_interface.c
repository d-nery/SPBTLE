#include "bluenrg_interface.h"
#include "spi.h"

#include "bluenrg_aci_const.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_utils.h"
#include "debug.h"
#include "gp_timer.h"
#include "hal.h"
#include "hal_types.h"
#include "hci.h"
#include "hci_const.h"
#include "hci_le.h"
#include "osal.h"
#include "sm.h"
#include "string.h"

#include "ble_clock.h"
#include "ble_status.h"
#include "gp_timer.h"

#define HEADER_SIZE 5
#define MAX_BUFFER_SIZE 255
#define TIMEOUT_DURATION 15
#define BDADDR_SIZE 6

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, \
                      uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0)                                    \
    do {                                                                                                         \
        uuid_struct[0] = uuid_0;                                                                                 \
        uuid_struct[1] = uuid_1;                                                                                 \
        uuid_struct[2] = uuid_2;                                                                                 \
        uuid_struct[3] = uuid_3;                                                                                 \
        uuid_struct[4] = uuid_4;                                                                                 \
        uuid_struct[5] = uuid_5;                                                                                 \
        uuid_struct[6] = uuid_6;                                                                                 \
        uuid_struct[7] = uuid_7;                                                                                 \
        uuid_struct[8] = uuid_8;                                                                                 \
        uuid_struct[9] = uuid_9;                                                                                 \
        uuid_struct[10] = uuid_10;                                                                               \
        uuid_struct[11] = uuid_11;                                                                               \
        uuid_struct[12] = uuid_12;                                                                               \
        uuid_struct[13] = uuid_13;                                                                               \
        uuid_struct[14] = uuid_14;                                                                               \
        uuid_struct[15] = uuid_15;                                                                               \
    } while (0)

#define COPY_LED_SERVICE_UUID(uuid_struct)                                                                         \
    COPY_UUID_128(uuid_struct, 0x0b, 0x36, 0x6e, 0x80, 0xcf, 0x3a, 0x11, 0xe1, 0x9a, 0xb4, 0x00, 0x02, 0xa5, 0xd5, \
                  0xc5, 0x1b)
#define COPY_LED_UUID(uuid_struct)                                                                                 \
    COPY_UUID_128(uuid_struct, 0x0c, 0x36, 0x6e, 0x80, 0xcf, 0x3a, 0x11, 0xe1, 0x9a, 0xb4, 0x00, 0x02, 0xa5, 0xd5, \
                  0xc5, 0x1b)

#define COPY_MY_SERVICE_UUID(uuid_struct)                                                                          \
    COPY_UUID_128(uuid_struct, 0x8c, 0x6d, 0x37, 0x20, 0xab, 0x19, 0x11, 0xe8, 0xb5, 0x68, 0x08, 0x00, 0x20, 0x0c, \
                  0x9a, 0x66);
#define COPY_MY_CHAR_UUID(uuid_struct)                                                                             \
    COPY_UUID_128(uuid_struct, 0x45, 0xf6, 0x92, 0x40, 0xab, 0xce, 0x11, 0xe8, 0xb5, 0x68, 0x08, 0x00, 0x20, 0x0c, \
                  0x9a, 0x66);

static void us150Delay();

uint16_t ledServHandle, ledButtonCharHandle;
uint8_t ledState = 0;

uint16_t myServHandle;
uint16_t myCharHandle;

volatile uint8_t set_connectable = 1;
volatile int connected;

/**
 * BlueNRG_Init()
 * Initilizes the SPBTLE-RF
 */
tBleStatus BlueNRG_Init() {
    const char* name = "BLE_TR";
    uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
    uint8_t bdaddr[BDADDR_SIZE];
    uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

    uint8_t hwVersion;
    uint16_t fwVersion;

    tBleStatus ret;

    HCI_Init();
    BlueNRG_RST();

    PRINTF("Getting BNRG Version...\n");
    if ((ret = getBlueNRGVersion(&hwVersion, &fwVersion)) != BLE_STATUS_SUCCESS) {
        PRINTF("Get BNRG version failed\n");
        return ret;
    } else {
        PRINTF("HWver %d, FWver %d\n", hwVersion, fwVersion);
    }

    BlueNRG_RST();

    // Get bdaddr from system memory
    Osal_MemCpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));

    // Set BLE MAC address
    if ((ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr)) !=
        BLE_STATUS_SUCCESS) {
        PRINTF("Setting bdaddr failed.\n");
        return ret;
    } else {
        PRINTF("Setting bdaddr success.\n");
    }

    if ((ret = aci_gatt_init()) != BLE_STATUS_SUCCESS) {
        PRINTF("GATT init failed.\n");
        return ret;
    } else {
        PRINTF("GATT init success.\n");
    }

    if ((ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, strlen(name), &service_handle,
                                    &dev_name_char_handle, &appearance_char_handle)) != BLE_STATUS_SUCCESS) {
        PRINTF("GAP init failed.\n");
        return ret;
    } else {
        PRINTF("GAP init success.\n");
    }

    if ((ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen(name), (uint8_t*) name)) !=
        BLE_STATUS_SUCCESS) {
        PRINTF("aci_gatt_update_char_value failed.\n");
        return ret;
    } else {
        PRINTF("aci_gatt_update_char_value success.\n");
    }

    if ((ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED, OOB_AUTH_DATA_ABSENT, NULL, 7, 16,
                                            USE_FIXED_PIN_FOR_PAIRING, 123456, BONDING)) == BLE_STATUS_SUCCESS) {
        PRINTF("BLE stack initialized!\n");
    } else {
        PRINTF("Error initializing :(\n");
        return ret;
    }

    // Add services here

    // Select the transmitting power level
    ret = aci_hal_set_tx_power_level(1, 4);
    return ret;
}

/**
 * @brief  Set in Output mode the IRQ.
 * @param  None
 * @retval None
 */
void set_irq_as_output() {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Pull IRQ high */
    GPIO_InitStructure.Pin = BNRG_SPI_IRQ_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = BNRG_SPI_IRQ_SPEED;
    GPIO_InitStructure.Pull = BNRG_SPI_IRQ_PULL;
    HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);
    HAL_GPIO_WritePin(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN, GPIO_PIN_SET);
}

void set_irq_as_input() {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* IRQ input */
    GPIO_InitStructure.Pin = BNRG_SPI_IRQ_PIN;
    GPIO_InitStructure.Mode = BNRG_SPI_IRQ_MODE;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed = BNRG_SPI_IRQ_SPEED;
    GPIO_InitStructure.Alternate = BNRG_SPI_IRQ_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);
}

void BlueNRG_RST(void) {
    HAL_GPIO_WritePin(BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
}

void Hal_Write_Serial(const void* data1, const void* data2, int32_t n_bytes1, int32_t n_bytes2) {
    struct timer t;

    Timer_Set(&t, CLOCK_SECOND / 10);

    while (1) {
        if (BlueNRG_SPI_Write((uint8_t*) data1, (uint8_t*) data2, n_bytes1, n_bytes2) == 0)
            break;
        if (Timer_Expired(&t)) {
            break;
        }
    }
}

uint8_t BlueNRG_DataPresent() {
    if (HAL_GPIO_ReadPin(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN) == GPIO_PIN_SET)
        return 1;
    else
        return 0;
}

int32_t BlueNRG_SPI_Read_All(uint8_t* buffer, uint8_t buff_size) {
    uint16_t byte_count;
    uint8_t len = 0;
    uint8_t char_ff = 0xff;
    volatile uint8_t read_char;

    uint8_t header_master[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
    uint8_t header_slave[HEADER_SIZE];

    /* CS reset */
    HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);

    /* Read the header */
    if (header_slave[0] == 0x02) {
        /* device is ready */
        byte_count = (header_slave[4] << 8) | header_slave[3];

        if (byte_count > 0) {
            /* avoid to read more data that size of the buffer */
            if (byte_count > buff_size) {
                byte_count = buff_size;
            }

            for (len = 0; len < byte_count; len++) {
            }
        }
    }
    /* Release CS line */
    HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);

    // Add a small delay to give time to the BlueNRG to set the IRQ pin low
    // to avoid a useless SPI read at the end of the transaction
    for (volatile int i = 0; i < 2; i++)
        __NOP();

    return len;
}

int32_t BlueNRG_SPI_Write(uint8_t* data1, uint8_t* data2, uint8_t Nb_bytes1, uint8_t Nb_bytes2) {
    int32_t result = 0;
    int32_t spi_fix_enabled = 0;

    // #ifdef ENABLE_SPI_FIX
    spi_fix_enabled = 1;
    // #endif //ENABLE_SPI_FIX

    unsigned char header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
    unsigned char header_slave[HEADER_SIZE] = {0xaa, 0x00, 0x00, 0x00, 0x00};

    unsigned char read_char_buf[MAX_BUFFER_SIZE];

    Disable_SPI_IRQ();

    /*
    If the SPI_FIX is enabled the IRQ is set in Output mode, then it is pulled
    high and, after a delay of at least 112us, the CS line is asserted and the
    header transmit/receive operations are started.
    After these transmit/receive operations the IRQ is reset in input mode.
    */
    if (spi_fix_enabled) {
        set_irq_as_output();

        /* Assert CS line after at least 112us */
        us150Delay();
    }

    /* CS reset */
    HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);

    /* Exchange header */
    HAL_SPI_TransmitReceive(&BNRG_SPI_HANDLER, header_master, header_slave, HEADER_SIZE, TIMEOUT_DURATION);

    if (spi_fix_enabled) {
        set_irq_as_input();
    }

    if (header_slave[0] == 0x02) {
        /* SPI is ready */
        if (header_slave[1] >= (Nb_bytes1 + Nb_bytes2)) {
            /*  Buffer is big enough */
            if (Nb_bytes1 > 0) {
                HAL_SPI_TransmitReceive(&BNRG_SPI_HANDLER, data1, read_char_buf, Nb_bytes1, TIMEOUT_DURATION);
            }
            if (Nb_bytes2 > 0) {
                HAL_SPI_TransmitReceive(&BNRG_SPI_HANDLER, data2, read_char_buf, Nb_bytes2, TIMEOUT_DURATION);
            }

        } else {
            /* Buffer is too small */
            result = -2;
        }
    } else {
        /* SPI is not ready */
        result = -1;
    }

    /* Release CS line */
    HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);

    Enable_SPI_IRQ();

    return result;
}

static void us150Delay(void) {
    for (volatile int i = 0; i < 750; i++)
        __NOP();
}

void Enable_SPI_IRQ(void) {
    HAL_NVIC_EnableIRQ(BNRG_SPI_EXTI_IRQn);
}

void Disable_SPI_IRQ(void) {
    HAL_NVIC_DisableIRQ(BNRG_SPI_EXTI_IRQn);
}

void Clear_SPI_IRQ(void) {
    HAL_NVIC_ClearPendingIRQ(BNRG_SPI_EXTI_IRQn);
}

void Clear_SPI_EXTI_Flag(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(BNRG_SPI_EXTI_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    HCI_Isr();
}

tBleStatus Add_LED_Service() {
    tBleStatus ret;
    uint8_t uuid[16];

    /* copy "LED service UUID" defined above to 'uuid' local variable */
    COPY_LED_SERVICE_UUID(uuid);
    /*
     * now add "LED service" to GATT server, service handle is returned
     * via 'ledServHandle' parameter of aci_gatt_add_serv() API.
     * Please refer to 'BlueNRG Application Command Interface.pdf' for detailed
     * API description
     */
    ret = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 7, &ledServHandle);
    if (ret != BLE_STATUS_SUCCESS)
        goto fail;

    /* copy "LED button characteristic UUID" defined above to 'uuid' local variable */
    COPY_LED_UUID(uuid);
    /*
     * now add "LED button characteristic" to LED service, characteristic handle
     * is returned via 'ledButtonCharHandle' parameter of aci_gatt_add_char() API.
     * This characteristic is writable, as specified by 'CHAR_PROP_WRITE' parameter.
     * Please refer to 'BlueNRG Application Command Interface.pdf' for detailed
     * API description
     */
    ret = aci_gatt_add_char(ledServHandle, UUID_TYPE_128, uuid, 4, CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
                            ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 1, &ledButtonCharHandle);

    if (ret != BLE_STATUS_SUCCESS)
        goto fail;

    PRINTF("Service LED BUTTON added. Handle 0x%04X, LED button Charac handle: 0x%04X\n", ledServHandle,
           ledButtonCharHandle);
    return BLE_STATUS_SUCCESS;

fail:
    PRINTF("Error while adding LED service.\n");
    return BLE_STATUS_ERROR;
}

/* Adds a new GATT service to BLE device */
tBleStatus Add_My_Service() {
    tBleStatus ret;
    uint8_t uuid[16];
    uint8_t char_uuid[16];

    ret = BLE_STATUS_SUCCESS;

    // Get the service UUID defined above (generated randomly) to a local uuid variable
    COPY_MY_SERVICE_UUID(uuid);
    // Create the primary service, returns 'myServHandle' handler
    ret = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 7, &myServHandle);
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Error while adding new Service");
        return ret;
    }

    // Get characteristic UUID defined above to local uuid variable
    COPY_MY_CHAR_UUID(char_uuid);
    // Add GATT characteristic, returns 'myCharHandle' handler
    ret = aci_gatt_add_char(myServHandle, UUID_TYPE_128, char_uuid, 20,
                            CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_READ, ATTR_PERMISSION_NONE,
                            GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 1, &myCharHandle);

    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Error while adding new characteristic");
        return ret;
    }

    /* More characteristics could be added here */

    return ret;
}

void User_Process() {
    if (set_connectable) {
        setConnectable();
        set_connectable = FALSE;
    }
}

void setConnectable(void) {
    tBleStatus ret;

    const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME, 'B', 'l', 'u', 'e', 'N', 'R', 'G', '-', 'T', 'R'};

    /* disable scan response */
    hci_le_set_scan_resp_data(0, NULL);
    PRINTF("General Discoverable Mode.\n");

    ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0,
                                   NULL, 0, 0);
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Error while setting discoverable mode (%d)\n", ret);
        Error_Handler();
    }
}

void HCI_Event_CB(void* pckt) {
    hci_uart_pckt* hci_pckt = pckt;
    /* obtain event packet */
    hci_event_pckt* event_pckt = (hci_event_pckt*) hci_pckt->data;

    if (hci_pckt->type != HCI_EVENT_PKT)
        return;

    switch (event_pckt->evt) {
        case EVT_VENDOR: {
            evt_blue_aci* blue_evt = (void*) event_pckt->data;
            switch (blue_evt->ecode) {
                case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED: {
                    /* this callback is invoked when a GATT attribute is modified
                    extract callback data and pass to suitable handler function */
                    evt_gatt_attr_modified_IDB05A1* evt = (evt_gatt_attr_modified_IDB05A1*) blue_evt->data;
                    // Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
                    if (evt->attr_handle == ledButtonCharHandle + 1) {
                        // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                        led_toggle();
                    } else if (evt->attr_handle == myCharHandle + 1) {
                        bt_recv = evt->att_data[evt->offset];
                    }
                } break;
            }
        } break;
    }
}

tBleStatus Update_Charac(uint8_t value) {
    tBleStatus ret;

    ret = aci_gatt_update_char_value(myServHandle, myCharHandle, 0, 1, &value);

    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Failed to update characteristic value\n");
        return ret;
    }

    return BLE_STATUS_SUCCESS;
}
