
#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pid_ctrl.h"
#include "esp_random.h"
#include <math.h>
#include "pressure.h"

// Sensor variables
static float adc_voltage;
static float sensor_voltage;
static float pressure_mpa;
static QueueHandle_t pressure_queue;

static const char *TAG = "PID_CONTROLLER";

// PID settings
#define PID_LOOP_PERIOD_MS  100

// Desired flow rate
static float target_flow = 100.0f;

// Stepper motor output limits
#define MOTOR_MIN_FREQ  0.0f
#define MOTOR_MAX_FREQ  5000.0f

// PID controller handle
static pid_ctrl_block_handle_f_t pid_ctrl = NULL;

// Motor Control
void set_motor_frequency(float frequency)
{
    static float previous_frequency = 0.0f;

    // Simulate motor response
    float expected_flow = frequency * FLOW_PER_HZ;

    // Gradual response instead of instantaneous flow changes
    simulated_flow += 0.3f * (expected_flow - simulated_flow);

    // Random disturbance: 5% chance every PID cycle
    uint32_t random_value = esp_random() % 100;

    if (random_value < DISTURBANCE_PROBABILITY)
    {
        // Random disturbance between -30 and +30 mL/hr
        int32_t disturbance =
            (int32_t)(esp_random() % 61) - 30;

        simulated_flow += (float)disturbance;

        ESP_LOGW(TAG,
                 "Random disturbance: %ld mL/hr",
                 (long)disturbance);
    }

    // Prevent negative simulated flow
    if (simulated_flow < 0.0f)
    {
        simulated_flow = 0.0f;
    }

    previous_frequency = frequency;

    ESP_LOGI(TAG,
             "Motor: %.2f Hz | Simulated flow: %.2f mL/hr",
             frequency,
             simulated_flow);
}

// PID Init
void pid_initialize(void)
{
    pid_ctrl_parameter_f_t pid_parameters = {
        .kp = 70.0f,
        .ki = 10.0f,
        .kd = 5.1f,

        .cal_type = PID_CAL_TYPE_INCREMENTAL,

        .max_output = MOTOR_MAX_FREQ,
        .min_output = MOTOR_MIN_FREQ,

        .max_integral = 1000.0f,
        .min_integral = -1000.0f
    };

    pid_ctrl_config_f_t pid_config = {
        .init_param = pid_parameters
    };

    ESP_ERROR_CHECK(
        pid_new_control_block(&pid_config, &pid_ctrl)
    );

    ESP_LOGI(TAG, "PID initialized");
}


// PID Tasks
void pid_task(void *pvParameters)
{
    float measured_flow;
    float error;
    float motor_frequency = 0.0f;

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        // Read actual flow rate
        measured_flow = xQueuePeek(
            pressure_queue,
            &pressure_mpa,
            0
        );

        // Calculate error
        error = target_flow - measured_flow;

        // Compute PID output
        ESP_ERROR_CHECK(
            pid_compute(pid_ctrl, error, &motor_frequency)
        );

        // Apply output to motor
        set_motor_frequency(motor_frequency);

        // Print monitoring information
        ESP_LOGI(TAG,
                "SP: %.2f | PV: %.2f | Error: %.2f | Motor: %.2f Hz",
                target_flow,
                measured_flow,
                error,
                motor_frequency);

        // Maintain a fixed 100 ms loop period
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(PID_LOOP_PERIOD_MS)
        );
    }
}

// Sensor task
void sensor_task(void *pvParameters)
{
    while (1)
    {
        esp_err_t ret = ads1115_read_voltage(&adc_voltage);

        if (ret == ESP_OK)
        {
            sensor_voltage = pressure_sensor_get_voltage(adc_voltage);      // Compensate for voltage divider
            pressure_mpa = pressure_sensor_get_mpa(sensor_voltage);         // Convert to pressure

            xQueueOverwrite(pressure_queue, &pressure_mpa);
        }
        else
        {
            ESP_LOGE(TAG, "ADS1115 read failed: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        
    }
}


void app_main(void)
{
    // Initialize 
    pid_initialize();
    i2c_master_init();
    ads1115_init();
    pressure_queue = xQueueCreate(1, sizeof(float));

    // Start task
    xTaskCreate(
        pid_task,
        "pid_task",
        4096,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        sensor_task,
        "sensor_task",
        4096,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        print_data_task,
        "print_data_task",
        4096,
        NULL,
        5,
        NULL
    );
}
