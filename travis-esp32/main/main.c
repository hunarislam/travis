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
#include <time.h>
#include <math.h>

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
float kp_left = 15.0;
float ki_left = 03.5;
float kd_left = 64.0;

float kp_right = 15.0;
float ki_right = 03.5;
float kd_right = 64.0;

// Encoder Constants
float wheelbase = 0.284;
float wheel_dia = 0.10;
bool direction_left = 0; // Forward
bool direction_right = 1; // Forward

float accel_limit = 0.3;

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
unsigned long current_cmd_vel_time;
unsigned long last_cmd_vel_time;
float current_cmd_vel_left;
float current_cmd_vel_right;
float last_cmd_vel_left;
float last_cmd_vel_right;

unsigned long millis() {
    struct timespec currentTime;
    clock_gettime(CLOCK_REALTIME, &currentTime);

    unsigned long milliseconds = currentTime.tv_sec * 1000 + currentTime.tv_nsec / 1000000;
    return milliseconds;
}

// Helper functions to limit a value
float limitToMinimum(float value, float minimum) {
  return (value < minimum) ? minimum : value;
}

float limitToMaximum(float value, float maximum) {
  return (value > maximum) ? maximum : value;
}

void updatePID(float current_velocity_left, float current_velocity_right) {
  // Calculate the error between the desired setpoint and the current velocity for each system
  error_left = fabs(setpoint_left) - fabs(current_velocity_left);
  error_right = fabs(setpoint_right) - fabs(current_velocity_right);

  // Calculate the integral of the error for each system
  integral_left += error_left;
  integral_right += error_right;

  // Calculate the derivative of the error for each system
  derivative_left = error_left - prev_error_left;
  derivative_right = error_right - prev_error_right;

  // Calculate the duty cycle (control output) for each system using the PID formula
  duty_cycle_left = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
  duty_cycle_right = kp_right * error_right + ki_right * integral_right + kd_right * derivative_right;

  // Limit the duty cycle
  duty_cycle_left = limitToMinimum(duty_cycle_left, 0);
  duty_cycle_right = limitToMinimum(duty_cycle_right, 0);

  // Update the previous error for each system
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
    // int32_t pwm_duty_left = (uint32_t)(duty_cycle_left * 65535.0);
    // int32_t pwm_duty_right = (uint32_t)(duty_cycle_right * 65535.0);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle_left);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle_right);

    //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); 
}

float calculateNewVelocity(float setpoint, float dt, float last_velocity)
{
  float max_acceleration = accel_limit * dt;
  float error = setpoint - last_velocity;
  float new_velocity = last_velocity;

  if (error > 0) 
  { // Need to accelerate
    float acceleration = limitToMaximum(error, max_acceleration);
    new_velocity += acceleration;
  }
  else if (error < 0) 
  { // Need to decelerate
    float acceleration = limitToMaximum(-error, max_acceleration);
    new_velocity -= acceleration;
  }

  return new_velocity;
}

static void updateCmdWheelVelocities(void)
{
    float cmd_vel_x, cmd_vel_z;
    current_cmd_vel_time = millis();
    float dt = (float)current_cmd_vel_time - (float)last_cmd_vel_time;
    getCmdVel(&cmd_vel_x, &cmd_vel_z);    
    // calculate left and right wheel velocities from angular z and linear x velocities
    setpoint_left = calculateNewVelocity((cmd_vel_x - cmd_vel_z * wheelbase / 2.0), dt, last_cmd_vel_left);
    setpoint_right = calculateNewVelocity((cmd_vel_x + cmd_vel_z * wheelbase / 2.0), dt, last_cmd_vel_right);
    last_cmd_vel_left = setpoint_left == -0.0 ? 0.0 : setpoint_left;
    last_cmd_vel_right = setpoint_right == -0.0 ? 0.0 : setpoint_right;
    
    last_cmd_vel_time = current_cmd_vel_time;
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
    unsigned long last_time = 0;
    unsigned long current_time = 0;
    unsigned long time_diff = 0;
    float wheel_circumference = wheel_dia * 3.14159;
    int encoder_resolution = 300;
    float ticks_per_second_left = 0.0;
    float ticks_per_second_right = 0.0;
    while (1)
    {   
        encoder_left_val = encoder_left->get_counter_value(encoder_left);
        encoder_right_val = encoder_right->get_counter_value(encoder_right);

        current_time = millis();
        time_diff = (float)current_time - (float)last_time;

        ticks_per_second_left = 1000 * (float)(encoder_left_val - encoder_left_prev_val) / (float)time_diff;
        ticks_per_second_right = 1000 * (float)(encoder_right_val - encoder_right_prev_val) / (float)time_diff;
        vel_left = (wheel_circumference * ticks_per_second_left) / (float)encoder_resolution; // convert endcoder value to wheel velocity in m/s
        vel_right = (wheel_circumference * ticks_per_second_right) / (float)encoder_resolution; // convert endcoder value to wheel velocity in m/s

        last_time = current_time;

        ESP_LOGI(TAG, "%f, %f, %f, %f", vel_left, vel_right, error_right, duty_cycle_right);

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
