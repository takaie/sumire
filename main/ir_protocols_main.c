/* IR protocols example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "ir_tools.h"


#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "ir_aeha.h"


#define RMT_TX_CHANNEL    1     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM  17     /*!< GPIO number for transmitter signal */
#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM  34     /*!< GPIO number for receiver */

static const char *TAG = "example";

static rmt_channel_t example_tx_channel = RMT_CHANNEL_0;
static rmt_channel_t example_rx_channel = RMT_CHANNEL_1;

/**
 * @brief RMT Receive Task
 *
 */
static void example_ir_rx_task(void *arg)
{
    uint32_t addr = 0;
    uint32_t cmd = 0;
    uint32_t length = 0;
    bool repeat = false;
    RingbufHandle_t rb = NULL;
    rmt_item32_t *items = NULL;

    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(CONFIG_EXAMPLE_RMT_RX_GPIO, example_rx_channel);
    rmt_config(&rmt_rx_config);
    rmt_driver_install(example_rx_channel, 1000, 0);
    ir_parser_config_t ir_parser_config = IR_PARSER_DEFAULT_CONFIG((ir_dev_t)example_rx_channel);
    ir_parser_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT; // Using extended IR protocols (both NEC and RC5 have extended version)
    ir_parser_t *ir_parser = NULL;
#if CONFIG_EXAMPLE_IR_PROTOCOL_NEC
    ir_parser = ir_parser_rmt_new_nec(&ir_parser_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_RC5
    ir_parser = ir_parser_rmt_new_rc5(&ir_parser_config);
#endif

    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(example_rx_channel, &rb);
    // Start receive
    rmt_rx_start(example_rx_channel, true);
    while (rb) {
        items = (rmt_item32_t *) xRingbufferReceive(rb, &length, 1000);
        if (items) {
            length /= 4; // one RMT = 4 Bytes
            if (ir_parser->input(ir_parser, items, length) == ESP_OK) {
                if (ir_parser->get_scan_code(ir_parser, &addr, &cmd, &repeat) == ESP_OK) {
                    ESP_LOGI(TAG, "Scan Code %s --- addr: 0x%04x cmd: 0x%04x", repeat ? "(repeat)" : "", addr, cmd);
                }
            }
            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void *) items);
        } else {
            break;
        }
    }
    ir_parser->del(ir_parser);
    rmt_driver_uninstall(example_rx_channel);
    vTaskDelete(NULL);
}

/**
 * @brief RMT Transmit Task
 *
 */
static void example_ir_tx_task(void *arg)
{
    uint32_t addr = 0x10;
    uint32_t cmd = 0x20;
    rmt_item32_t *items = NULL;
    uint32_t length = 0;
    ir_builder_t *ir_builder = NULL;

    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, example_tx_channel);
    rmt_tx_config.tx_config.carrier_en = true;
    rmt_config(&rmt_tx_config);
    rmt_driver_install(example_tx_channel, 0, 0);
    ir_builder_config_t ir_builder_config = IR_BUILDER_DEFAULT_CONFIG((ir_dev_t)example_tx_channel);
    ir_builder_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT; // Using extended IR protocols (both NEC and RC5 have extended version)
#if CONFIG_EXAMPLE_IR_PROTOCOL_NEC
    ir_builder = ir_builder_rmt_new_nec(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_RC5
    ir_builder = ir_builder_rmt_new_rc5(&ir_builder_config);
#endif
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "Send command 0x%x to address 0x%x", cmd, addr);
        // Send new key code
        ESP_ERROR_CHECK(ir_builder->build_frame(ir_builder, addr, cmd));
        ESP_ERROR_CHECK(ir_builder->get_result(ir_builder, &items, &length));
        //To send data according to the waveform items.
        rmt_write_items(example_tx_channel, items, length, true);
        // Send repeat code
        vTaskDelay(pdMS_TO_TICKS(ir_builder->repeat_period_ms));
        ESP_ERROR_CHECK(ir_builder->build_repeat_frame(ir_builder));
        ESP_ERROR_CHECK(ir_builder->get_result(ir_builder, &items, &length));
        rmt_write_items(example_tx_channel, items, length, true);
        cmd++;
    }
    ir_builder->del(ir_builder);
    rmt_driver_uninstall(example_tx_channel);
    vTaskDelete(NULL);
}

static void rmt_example_aeha_rx_task()
{
    int channel = RMT_RX_CHANNEL;
    aeha_rx_init(RMT_RX_CHANNEL, RMT_RX_GPIO_NUM);
    RingbufHandle_t rb = NULL;
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);
    while(rb) {
        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        ESP_LOGI(TAG, "xRingbufferReceive rx_size=%d", rx_size);
        if(item) {
            ESP_LOGI(TAG, "RMT RCV rx_size=%d", rx_size);
            uint16_t customer;
            uint8_t parity;
            uint8_t data[32];
            int offset = 0;
            int index = 0;
            while(1) {
                //parse data value from ringbuffer.
                //int res = aeha_parse_items(item + offset, rx_size / 4 - offset, &customer, data);
                //int res = aeha_parse_items(item + offset, rx_size / sizeof(rmt_item32_t) - offset, &customer, &parity, &index, data);
                int item_num = rx_size / sizeof(rmt_item32_t) - offset;
                int res = aeha_parse_items(item + offset, item_num, &customer, &parity, &index, data);
                ESP_LOGI(TAG, "RMT RCV offset=%d item_num=%d res=%d", offset, item_num, res);
                if(res > 0) {
                    offset += res + 1;
                    ESP_LOGI(TAG, "RMT RCV --- customer: 0x%04x parity: 0x%02x index: %d", customer, parity, index);
                    for(int i=0; i<index; i++) {
                        ESP_LOGI(TAG, "RMT RCV --- data[%d]: 0x%02x", i, data[i]);
                    }
                } else {
                    break;
                }
            }
            //after parsing the data, return spaces to ringbuffer.
            ESP_LOGI(TAG, "vRingbufferReturnItem");
            vRingbufferReturnItem(rb, (void*) item);
#if 0
        } else {
            break;
#endif
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief RMT transmitter demo, this task will periodically send AEHA data.
 *
 */
static void rmt_example_aeha_tx_task()
{
    vTaskDelay(10);
    aeha_tx_init(RMT_TX_CHANNEL, RMT_TX_GPIO_NUM);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    int channel = RMT_TX_CHANNEL;
    uint16_t customer = 0x2002;
    uint8_t parity = 0x00;
    uint8_t data[10];
    data[0] = 0x08;
    data[1] = 0x00;
    data[2] = 0x3d;
    data[3] = 0xbd;
    int data_num = 4;
    //int nec_tx_num = RMT_TX_DATA_NUM;
    for(;;) {
        ESP_LOGI(TAG, "RMT TX DATA");
        size_t size = (sizeof(rmt_item32_t) * (AEHA_DATA_ITEM_NUM + (data_num*8)));
        //each item represent a cycle of waveform.
        rmt_item32_t* item = (rmt_item32_t*) malloc(size);
        //int item_num = NEC_DATA_ITEM_NUM * nec_tx_num;
        memset((void*) item, 0, size);
        //int i, offset = 0;

        //To build a series of waveforms.
        int item_num = aeha_build_items(channel, item, data_num, customer, parity, data);
        //To send data according to the waveform items.
        rmt_write_items(channel, item, item_num, true);
        //Wait until sending is done.
        rmt_wait_tx_done(channel, portMAX_DELAY);
        //before we free the data, make sure sending is already done.
        free(item);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(rmt_example_aeha_rx_task, "rmt_rx_task", 2048, NULL, 10, NULL);
    //xTaskCreate(rmt_example_aeha_tx_task, "rmt_tx_task", 2048, NULL, 10, NULL);
//    xTaskCreate(example_ir_rx_task, "ir_rx_task", 2048, NULL, 10, NULL);
//    xTaskCreate(example_ir_tx_task, "ir_tx_task", 2048, NULL, 10, NULL);
}
