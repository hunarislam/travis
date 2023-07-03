#include "stdio.h"
#include "esp_chatter.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rotary_encoder.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/gpio.h"

static const char *TAG = "Wheel Velocities";
   

///////PIN BINDINGS///////
// Left Motor         ////
// DIR:  D12          ////
// EncA: D14          ////
// PWM:  D13          ////
// EncB: D27          ////
//////////////////////////
// Right Motor        ////
// PWM:  D15          ////
// DIR:  D2           ////
// EncA: D23          ////
// EncB: D22          ////
//////////////////////////

// PID Constants
float kp_left = 4;
float ki_left = 0;
float kd_left = 0;

float kp_right = 3.7;
float ki_right = 0;
float kd_right = 0;

// Encoder Constants
float wheelbase = 0.284;
bool direction_left = 0; // Forward
bool direction_right = 1; // Forward

#define GPIO_PWM_LEFT 13  //Set GPIO 13 as PWM0A / Left PWM
#define GPIO_PWM_RIGHT 15   //Set GPIO 15 as PWM0B / Right PWM
#define GPIO_DIR_LEFT GPIO_NUM_12
#define GPIO_DIR_RIGHT GPIO_NUM_2

// PID Variables
float error_left;
float error_right;
float prev_error_left;
float prev_error_right;
float integral_left;
float integral_right;
float derivative_left;
float derivative_right;
float duty_cycle_left;
float duty_cycle_right;
float setpoint_left;
float setpoint_right;
float vel_left, vel_right;


void updatePID(float current_velocity_left, float current_velocity_right) {
  error_left = setpoint_left - current_velocity_left;
  error_right = setpoint_right - current_velocity_right;
  integral_left += error_left;
  integral_right += error_right;
  derivative_left = error_left - prev_error_left;
  derivative_right = error_right - prev_error_right;
  duty_cycle_left = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
  duty_cycle_right = kp_right * error_right + ki_right * integral_right + kd_right * derivative_right;
  prev_error_left = error_left;
  prev_error_right = error_right;
}

static void mcpwmGpioInit()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM_LEFT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM_RIGHT);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[12], PIN_FUNC_GPIO);
    gpio_set_direction(GPIO_DIR_LEFT, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_DIR_RIGHT, GPIO_MODE_OUTPUT);
}

static void setMotorVelocity(
        mcpwm_unit_t mcpwm_num,
        mcpwm_timer_t timer_num,
        float duty_cycle_left,
        float duty_cycle_right,
        bool direction_left,
        bool direction_right
    )
{   
    // Set motor direction
    gpio_set_level(GPIO_DIR_LEFT, direction_left);
    gpio_set_level(GPIO_DIR_RIGHT, direction_right);

    // Set motor speed
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle_left);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle_right);

    //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); 
}

static void updateCmdWheelVelocities(void)
{
    float cmd_vel_x, cmd_vel_z;
    getCmdVel(&cmd_vel_x, &cmd_vel_z);    
    // calculate left and right wheel velocities from angular z and linear x velocities
    setpoint_left = (cmd_vel_x - cmd_vel_z * wheelbase / 2.0);
    setpoint_right = (cmd_vel_x + cmd_vel_z * wheelbase / 2.0);
}

static void motorController(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwmGpioInit();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    while (1) {
        updateCmdWheelVelocities();
        direction_left = (setpoint_left < 0) ? 1 : 0;
        direction_right = (setpoint_right < 0) ? 0 : 1;

        setMotorVelocity(MCPWM_UNIT_0, MCPWM_TIMER_0, duty_cycle_left, duty_cycle_right, direction_left, direction_right);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void encoderReader(void *arg){
    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit_enc_left = 0;
    uint32_t pcnt_unit_enc_right = 1;

    // Create rotary encoder instance
    rotary_encoder_config_t config_encoder_left = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_enc_left, 14, 27);
    rotary_encoder_config_t config_encoder_right = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_enc_right, 23, 22);

    rotary_encoder_t *encoder_left = NULL;
    rotary_encoder_t *encoder_right = NULL;

    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_encoder_left, &encoder_left));
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_encoder_right, &encoder_right));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder_left->set_glitch_filter(encoder_left, 1));
    ESP_ERROR_CHECK(encoder_right->set_glitch_filter(encoder_right, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder_left->start(encoder_left));
    ESP_ERROR_CHECK(encoder_right->start(encoder_right));

    // Setup rosserial
    rosserialSetup();

    // Report counter value
    int encoder_left_val = 0;
    int encoder_right_val = 0;
    int encoder_left_prev_val = 0;
    int encoder_right_prev_val = 0;
    vel_left = 0;
    vel_right = 0;
    while (1)
    {
        encoder_left_val = encoder_left->get_counter_value(encoder_left);
        encoder_right_val = encoder_right->get_counter_value(encoder_right);

        vel_left = 3.1415 * (encoder_left_val -  encoder_left_prev_val) / 30;
        vel_right = 3.1415 * (encoder_right_val - encoder_right_prev_val) / 30;
        
        ESP_LOGI(TAG, "%f, %f, %f", vel_left, vel_right, duty_cycle_left);

        float vel_x = (vel_left + vel_right) / 2.0;
        float vel_z = (vel_right - vel_left) / wheelbase;
        
        publishOdometry(vel_x, vel_z);

        updatePID(vel_left, vel_right); //m/s
        encoder_left_prev_val = encoder_left_val;
        encoder_right_prev_val = encoder_right_val;
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void odomPublisher(void)
{
    float vel_x = (vel_left + vel_right) / 2.0;
    float vel_z = (vel_right - vel_left) / wheelbase;

    publishOdometry(vel_x, vel_z);
    vTaskDelay(pdMS_TO_TICKS(100));
}
void app_main(void)
{   
    xTaskCreate(encoderReader, "encoderReader", 4096, NULL, 5, NULL);
    xTaskCreate(motorController, "motorController", 4096, NULL, 4, NULL);
    // TODO: check if odom publisher can run as a separate task
    // xTaskCreate(odomPublisher, "odomPublisher", 4096, NULL, 3, NULL);
}
