#include "idf-pmsx003.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define PMS_FRAME_LEN 32

#define PMS_PIN_VALUE_LOW 0

#define PMS_PIN_VALUE_HIGH 1

#define PMS_MIN_INTERVAL 35

#define PMS_WARMUP_DELAY  30000

#define PMS_WAIT_DATA_DELAY  1000

#define PMS_RESET_SENSOR_DELAY  1000

static const char* TAG = "pmsx003";

static const int pmsx_baud_rate = 9600;

static const int uart_buffer_size = 1024 * 2;

static TaskHandle_t xReadTaskHandles[UART_NUM_MAX] = { NULL };

static esp_timer_handle_t xReadTimerHandles[UART_NUM_MAX] = { NULL };

/*---------------- STATIC FUNCTIONS DEFs ----------------------------------*/
static esp_err_t pmsx_configure_gpio(pmsx003_config_t *config);

static esp_err_t pmsx_schedule_periodic_read(pmsx003_config_t *config);

static void pmsx_data_read_task();

static esp_err_t pms_uart_read(pmsx003_config_t *config, uint8_t *data);

static bool is_pmsx_frame_valid(uint8_t *data, int data_len);

static int calculate_frame_checksum(uint8_t *data);

static pm_data_t decode_pm_data(uint8_t *data, bool indoor);

static void pmsx_periodic_timer_callback(void* arg);
/*--------------------------------------------------------------------------*/

uint32_t idf_pmsx5003_init(pmsx003_config_t *config) {

    if (xReadTaskHandles[config->uart_port] != NULL) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "init pmsx sensor");

    uart_config_t uart_config = {
        .baud_rate = pmsx_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    esp_err_t result = pmsx_configure_gpio(config);
    if (result != ESP_OK)
        return ESP_FAIL;

    result = uart_param_config(config->uart_port, &uart_config);
    if (result != ESP_OK)
        return ESP_FAIL;
    
    result = uart_set_pin(config->uart_port, config->uart_tx_pin, config->uart_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (result != ESP_OK)
        return ESP_FAIL;
    
    result = uart_driver_install(config->uart_port, uart_buffer_size, uart_buffer_size, 0, NULL, 0);
    if (result != ESP_OK)
        return ESP_FAIL;

    if (config->periodic) {

        xTaskCreate(pmsx_data_read_task, "data read task", 4096, config, 5, &(xReadTaskHandles[config->uart_port]));
         
        result = pmsx_schedule_periodic_read(config);
        if (result != ESP_OK) {
            idf_pmsx5003_destroy(config);
            return ESP_FAIL; 
        }
    } else {
        xTaskCreate(pmsx_data_read_task, "data read task", 4096, config, 5, NULL);
    }
    
    ESP_LOGI(TAG, "pmsx sensor initialized successfully");

    return ESP_OK;
}

static esp_err_t pmsx_configure_gpio(pmsx003_config_t *config) {
    
    esp_rom_gpio_pad_select_gpio(config->set_pin);
    esp_err_t ret = gpio_set_direction(config->set_pin, GPIO_MODE_OUTPUT);
    ret += gpio_set_pull_mode(config->set_pin, GPIO_PULLDOWN_ONLY);
    ret += gpio_set_level(config->set_pin, PMS_PIN_VALUE_LOW);

    esp_rom_gpio_pad_select_gpio(config->reset_pin);
    ret += gpio_set_direction(config->reset_pin, GPIO_MODE_OUTPUT);
    ret += gpio_set_pull_mode(config->reset_pin, GPIO_PULLDOWN_ONLY);
    ret += gpio_set_level(config->reset_pin, PMS_PIN_VALUE_HIGH);

    return ret;
}

static esp_err_t pmsx_schedule_periodic_read(pmsx003_config_t *config) {

    const esp_timer_create_args_t pmsx_periodic_timer_args = {
        .callback = &pmsx_periodic_timer_callback,
        .arg = &config->uart_port,
        .name = "periodic pmsx timer task"
    };

    esp_err_t ret = esp_timer_create(&pmsx_periodic_timer_args, &(xReadTimerHandles[config->uart_port]));
    
    if (ret == ESP_OK) {
        int seconds = config->periodic_sec_interval > PMS_MIN_INTERVAL ? config->periodic_sec_interval : PMS_MIN_INTERVAL;
        uint64_t period = 1000000 * seconds;
        ret = esp_timer_start_periodic(xReadTimerHandles[config->uart_port], period);
    }

    return ret;
}

static void pmsx_data_read_task(pmsx003_config_t* config) {
    
    uint8_t data[PMS_FRAME_LEN];

    bool is_periodic = config->periodic;

    do {
        gpio_set_level(config->set_pin, PMS_PIN_VALUE_HIGH);
        vTaskDelay(PMS_WARMUP_DELAY / portTICK_PERIOD_MS);
        uart_flush(config->uart_port);
        vTaskDelay(PMS_WAIT_DATA_DELAY / portTICK_PERIOD_MS);
        
        if (config->enabled) {
            esp_err_t result_code = pms_uart_read(config, data);
            if (result_code != ESP_OK) {
                gpio_set_level(config->reset_pin, PMS_PIN_VALUE_LOW);
                vTaskDelay(PMS_RESET_SENSOR_DELAY / portTICK_PERIOD_MS);
                gpio_set_level(config->reset_pin, PMS_PIN_VALUE_HIGH);
            }
        }
        
        gpio_set_level(config->set_pin, PMS_PIN_VALUE_LOW);

        if (is_periodic) {
            vTaskSuspend(NULL);
        } else {
            idf_pmsx5003_destroy(config);
        }
        
    } while (is_periodic);
   
    vTaskDelete(NULL);
}

static esp_err_t pms_uart_read(pmsx003_config_t *config, uint8_t *data) {
	
    size_t available_data;
    uart_get_buffered_data_len(config->uart_port, &available_data);
    ESP_LOGI(TAG, "available=%d", available_data);
    if (available_data < PMS_FRAME_LEN) {
        ESP_LOGW(TAG, "not enough data available");
        return ESP_FAIL;
    }

    int data_len = uart_read_bytes(config->uart_port, data, PMS_FRAME_LEN, 100 / portTICK_PERIOD_MS);    
    if (is_pmsx_frame_valid(data, data_len)) {
        pm_data_t pm = decode_pm_data(data, config->indoor);
        pm.sensor_id = config->sensor_id;
        if (config->callback) {
            config->callback(&pm);
        }
    } else {
        ESP_LOGW(TAG, "checksum mismatch");
        return ESP_FAIL;
    }

	return ESP_OK;
}

static bool is_pmsx_frame_valid(uint8_t *data, int data_len) {

    if (data_len==PMS_FRAME_LEN && data[0]==0x42 && data[1]==0x4d) {
        
        int calculated_checksum = calculate_frame_checksum(data);
        int frame_checksum = ((data[30]<<8) + data[31]);
       
        return calculated_checksum == frame_checksum;
    }

    return false;
}

static int calculate_frame_checksum(uint8_t *data) {
    int sum = 0;
    for (uint8_t i=0; i<(PMS_FRAME_LEN -2); i++) {
        sum += data[i];
    }
    return sum;
}

static pm_data_t decode_pm_data(uint8_t* data, bool indoor) {
	
    uint8_t start_byte = indoor ? 4 : 10;
    
    pm_data_t pm = {
		.pm1_0 = ((data[start_byte]<<8) + data[start_byte+1]),
		.pm2_5 = ((data[start_byte+2]<<8) + data[start_byte+3]),
		.pm10 = ((data[start_byte+4]<<8) + data[start_byte+5]),
        .particles_03um = ((data[16]<<8) + data[17]),
        .particles_05um = ((data[18]<<8) + data[19]), 
        .particles_10um = ((data[20]<<8) + data[21]), 
        .particles_25um = ((data[22]<<8) + data[23]), 
        .particles_50um = ((data[24]<<8) + data[25]), 
        .particles_100um = ((data[26]<<8) + data[27])
	};
	return pm;
}

static void pmsx_periodic_timer_callback(void* arg) {
    
    uint8_t task_handle_index = *((uint8_t*) arg);
        
    TaskHandle_t task_handle = xReadTaskHandles[task_handle_index];
    if (task_handle != NULL) {
        vTaskResume(task_handle);
    }
}

void idf_pmsx5003_destroy(pmsx003_config_t* config) {

    ESP_LOGI(TAG, "stop pmsx sensor");

    if (xReadTaskHandles[config->uart_port] != NULL) {
        vTaskDelete(xReadTaskHandles[config->uart_port]);
    }
    
    if (xReadTimerHandles[config->uart_port] != NULL) {
        esp_timer_stop(xReadTimerHandles[config->uart_port]);
        esp_timer_delete(xReadTimerHandles[config->uart_port]);
    }
    
    xReadTaskHandles[config->uart_port] = NULL;
    xReadTimerHandles[config->uart_port] = NULL;
    
    uart_driver_delete(config->uart_port);
}