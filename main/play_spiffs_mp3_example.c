/* Play MP3 file from Flash(spiffs system)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "sdkconfig.h"

#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "i2s_stream.h"
#include "spiffs_stream.h"
#include "mp3_decoder.h"

#include "esp_peripherals.h"
#include "periph_spiffs.h"
#include "board.h"

#include "driver/rmt.h"
#include "led_strip.h"
#include "esp_pm.h"

static const char *TAG = "SPIFFS_MP3_EXAMPLE";
#define STACK_SIZE 12000


#define RMT_TX_CHANNEL RMT_CHANNEL_0

#define EXAMPLE_CHASE_SPEED_MS (20)

SemaphoreHandle_t xNuttonSemaphore;
/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

bool g_enable_blinking = false;

// Task to be created.
void vTaskSinger( void * pvParameters )
{
    for( ;; )
    {
        xSemaphoreTake( xNuttonSemaphore, portMAX_DELAY );
        g_enable_blinking = true;

        audio_pipeline_handle_t pipeline;
        audio_element_handle_t spiffs_stream_reader, i2s_stream_writer, mp3_decoder;

        esp_log_level_set("*", ESP_LOG_WARN);
        esp_log_level_set(TAG, ESP_LOG_INFO);

        // Initialize peripherals management
        esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
        esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

        ESP_LOGI(TAG, "[ 1 ] Mount spiffs");
        // Initialize Spiffs peripheral
        periph_spiffs_cfg_t spiffs_cfg = {
            .root = "/spiffs",
            .partition_label = NULL,
            .max_files = 5,
            .format_if_mount_failed = true
        };
        esp_periph_handle_t spiffs_handle = periph_spiffs_init(&spiffs_cfg);

        // Start spiffs
        esp_periph_start(set, spiffs_handle);

        // Wait until spiffs is mounted
        while (!periph_spiffs_is_mounted(spiffs_handle)) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        ESP_LOGI(TAG, "[ 2 ] Start codec chip");
        audio_board_handle_t board_handle = audio_board_init();
        audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

        ESP_LOGI(TAG, "[3.0] Create audio pipeline for playback");
        audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
        pipeline = audio_pipeline_init(&pipeline_cfg);
        AUDIO_NULL_CHECK(TAG, pipeline, return);

        ESP_LOGI(TAG, "[3.1] Create spiffs stream to read data from sdcard");
        spiffs_stream_cfg_t flash_cfg = SPIFFS_STREAM_CFG_DEFAULT();
        flash_cfg.type = AUDIO_STREAM_READER;
        spiffs_stream_reader = spiffs_stream_init(&flash_cfg);

        ESP_LOGI(TAG, "[3.2] Create i2s stream to write data to codec chip");
        //i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
        i2s_stream_cfg_t i2s_cfg = I2S_STREAM_INTERNAL_DAC_CFG_DEFAULT();
        i2s_cfg.type = AUDIO_STREAM_WRITER;
        i2s_stream_writer = i2s_stream_init(&i2s_cfg);

        ESP_LOGI(TAG, "[3.3] Create mp3 decoder to decode mp3 file");
        mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
        mp3_decoder = mp3_decoder_init(&mp3_cfg);

        ESP_LOGI(TAG, "[3.4] Register all elements to audio pipeline");
        audio_pipeline_register(pipeline, spiffs_stream_reader, "spiffs");
        audio_pipeline_register(pipeline, mp3_decoder, "mp3");
        audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

        ESP_LOGI(TAG, "[3.5] Link it together [flash]-->spiffs-->mp3_decoder-->i2s_stream-->[codec_chip]");
        const char *link_tag[3] = {"spiffs", "mp3", "i2s"};
        audio_pipeline_link(pipeline, &link_tag[0], 3);

        ESP_LOGI(TAG, "[3.6] Set up  uri (file as spiffs, mp3 as mp3 decoder, and default output is i2s)");
        audio_element_set_uri(spiffs_stream_reader, "/spiffs/aaa.mp3");

        ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
        audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
        audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

        ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
        audio_pipeline_set_listener(pipeline, evt);

        ESP_LOGI(TAG, "[4.2] Listening event from peripherals");
        audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

        ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
        audio_pipeline_run(pipeline);

        ESP_LOGI(TAG, "[ 6 ] Listen for all pipeline events");
        while (1) {
            audio_event_iface_msg_t msg;
            esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
                continue;
            }

            if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) mp3_decoder
                && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
                audio_element_info_t music_info = {0};
                audio_element_getinfo(mp3_decoder, &music_info);

                ESP_LOGI(TAG, "[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
                        music_info.sample_rates, music_info.bits, music_info.channels);

                audio_element_setinfo(i2s_stream_writer, &music_info);
                i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
                continue;
            }

            /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
            if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
                && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
                && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
                //ESP_LOGW(TAG, "[ * ] Stop event received");
                //break;
                audio_pipeline_reset_ringbuffer(pipeline);
                audio_pipeline_reset_elements(pipeline);
                audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
                audio_pipeline_run(pipeline);
            }
        }

        ESP_LOGI(TAG, "[ 7 ] Stop audio_pipeline");
        audio_pipeline_stop(pipeline);
        audio_pipeline_wait_for_stop(pipeline);
        audio_pipeline_terminate(pipeline);

        audio_pipeline_unregister(pipeline, spiffs_stream_reader);
        audio_pipeline_unregister(pipeline, i2s_stream_writer);
        audio_pipeline_unregister(pipeline, mp3_decoder);

        /* Terminal the pipeline before removing the listener */
        audio_pipeline_remove_listener(pipeline);

        /* Stop all periph before removing the listener */
        esp_periph_set_stop_all(set);
        audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

        /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
        audio_event_iface_destroy(evt);

        /* Release all resources */
        audio_pipeline_deinit(pipeline);
        audio_element_deinit(spiffs_stream_reader);
        audio_element_deinit(i2s_stream_writer);
        audio_element_deinit(mp3_decoder);
        esp_periph_set_destroy(set);

        xSemaphoreTake( xNuttonSemaphore, 0 );
        g_enable_blinking = false;

    }
}



void vTaskBlinker( void * pvParameters )
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_EXAMPLE_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "LED Rainbow Chase Start");
    while (true) {
        for (int i = 0; i < CONFIG_EXAMPLE_STRIP_LED_NUMBER; i++) {
            hue = i * (360 / CONFIG_EXAMPLE_STRIP_LED_NUMBER) + start_rgb;
            led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, i, red, green, blue));
        }
        start_rgb += 10;
        if (g_enable_blinking){
            ESP_ERROR_CHECK(strip->refresh(strip, 10)); 
        } else {
            strip->clear(strip, 10);
        }
        vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* un-block the interrupt processing task now */
    xSemaphoreGiveFromISR( xNuttonSemaphore, &xHigherPriorityTaskWoken );
}

#define LED_PIN 32
#define NUT_PIN1 26
#define NUT_PIN2 27
#define ESP_INTR_FLAG_DEFAULT 0

void configure_ios(){
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<NUT_PIN1);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(NUT_PIN1, gpio_isr_handler, (void*) NUT_PIN1);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<LED_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

}

void app_main(void)
{
    esp_pm_config_esp32_t pm_config;
    pm_config.max_freq_mhz = 80;
    pm_config.min_freq_mhz = 80;
    pm_config.light_sleep_enable = false;
    ESP_ERROR_CHECK( esp_pm_configure(&pm_config) );

    //TaskHandle_t xHandle = NULL;
    xNuttonSemaphore = xSemaphoreCreateBinary();

    //xTaskCreate( vTaskSinger, "SingerTask", STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
    //xTaskCreate( vTaskBlinker, "BlinkerTask", STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL );
    xTaskCreatePinnedToCore( vTaskSinger, "SingerTask", STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL ,1 );
    xTaskCreatePinnedToCore( vTaskBlinker, "BlinkerTask", STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL,0 );
    //configASSERT( xHandle );
    
    configure_ios();
    uint8_t cnt = 0;
    for( ;; )
    {
        gpio_set_level(LED_PIN, cnt++ % 2);
        vTaskDelay( 500 / portTICK_PERIOD_MS );
    }
}