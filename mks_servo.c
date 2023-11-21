/**
 * @file mks_servo.c
 * @author JanG175
 * @brief MKS SERVO SERIAL PROTOCOL LIBRARY
 * 
 * @copyright All rigths reserved (R) 2023
 */

#include <stdio.h>
#include "mks_servo.h"

static bool abort_on = true; // if true - abort on UART read timeout

#ifdef MKS_STEP_MODE_ENABLE
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static gptimer_handle_t* gptimer;

static mks_cb_arg_t* cb_arg;
#endif // MKS_STEP_MODE_ENABLE

static const char* TAG = "mks_servo";


/**
 * @brief calculate CRC for MKS UART communication
 * 
 * @param datagram pointer to datagram
 * @param len length of datagram
 * 
 * @return calculated CRC
 */
static uint8_t mks_servo_uart_calc_CRC(uint8_t* datagram, uint32_t len)
{
    uint64_t crc = 0;

    for (uint32_t i = 0; i < len - 1; i++)
        crc += datagram[i];

    return (uint8_t)(crc & 0xFF);
}


/**
 * @brief check if CRC is correct
 * 
 * @param datagram pointer to datagram
 * @param len length of datagram
 * 
 * @return true - if CRC is correct; false - otherwise
 */
static bool mks_servo_uart_check_CRC(uint8_t* datagram, uint32_t len)
{
    uint8_t crc = mks_servo_uart_calc_CRC(datagram, len);

    if (crc == datagram[len - 1])
        return true;
    else
        return false;
}


/**
 * @brief write datagram to register via UART
 * 
 * @param mks_config struct with MKS connection parameters
 * @param datagram pointer to datagram
 * @param len length of datagram
 */
static void mks_servo_uart_send(mks_conf_t mks_config, uint8_t* datagram, uint8_t len)
{
    uart_write_bytes(mks_config.uart, (const uint8_t*)datagram, len);
    ESP_ERROR_CHECK(uart_wait_tx_done(mks_config.uart, MKS_UART_TIMEOUT_MS));
}


/**
 * @brief read datagram from register via UART
 * 
 * @param mks_config struct with MKS connection parameters
 * @param datagram pointer to datagram
 * @param len length of datagram
 */
static void mks_servo_uart_recv(mks_conf_t mks_config, uint8_t* datagram, uint8_t len)
{
    uint32_t buf = 0;
    uint8_t data[len];

    buf = uart_read_bytes(mks_config.uart, data, len, MKS_UART_TIMEOUT_MS);
    uart_flush(mks_config.uart);

    // // for debug
    // printf("mks:\t");
    // for (uint32_t i = 0; i < buf; i++)
    //     printf("%2X ", data[i]);
    // printf("\n");

    if (buf == len)
    {
        for (int i = 0; i < buf; i++)
            datagram[i] = data[i];
    }
    else
    {
        ESP_LOGE(TAG, "UART read error");

        for (int i = 0; i < len; i++)
            datagram[i] = 0;
    }
}


/**
 * @brief check if UART response is correct
 * 
 * @param mks_config struct with MKS connection parameters
 * @param address MKS slave address
 * @param datagram pointer to send datagram
 * @param len_w length of send datagram
 * @param response pointer to response datagram
 * @param len_r length of response datagram
 */
static void mks_servo_uart_send_w_recv_check(mks_conf_t mks_config, uint8_t address, uint8_t* datagram, uint8_t len_w, uint8_t* response, uint8_t len_r)
{
    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_send(mks_config, datagram, len_w);
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address || mks_servo_uart_check_CRC(response, len_r) == false) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "%u: UART read timeout", address);

        if (abort_on == true)
            abort();
    }
}


#ifdef MKS_STEP_MODE_ENABLE
/**
 * @brief callback function for timers
 * 
 * @param timer timer handle
 * @param edata event data
 * @param user_ctx user context
 * @return true - if high priority task was woken up; false - otherwise
 */
static bool mks_servo_clk_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;

    portENTER_CRITICAL_ISR(&spinlock);
    mks_cb_arg_t arg = *(mks_cb_arg_t*)user_ctx;
    portEXIT_CRITICAL_ISR(&spinlock);

    if (arg.steps_left > 0)
    {
        if (gpio_get_level(arg.step_pin) == 1)
        {
            uint64_t period_cur = arg.period_goal;

            if ((double)(arg.steps_total - arg.steps_left) < arg.accel_s) // acceleration
                period_cur = (uint64_t)((arg.accel_s - (double)(arg.steps_total - arg.steps_left)) * arg.dt + (double)arg.period_goal);
            else if ((double)arg.steps_left < arg.accel_s) // deceleartion
                period_cur = (uint64_t)((arg.accel_s - (double)arg.steps_left) * arg.dt + (double)arg.period_goal);

            if (period_cur < arg.period_goal)
                period_cur = arg.period_goal;

            // change alert period
            mks_servo_set_period(arg.motor_num, period_cur);

            arg.time_passed += period_cur;
            arg.steps_left--;

            portENTER_CRITICAL_ISR(&spinlock);
            cb_arg[arg.motor_num].time_passed = arg.time_passed;
            cb_arg[arg.motor_num].steps_left = arg.steps_left;
            portEXIT_CRITICAL_ISR(&spinlock);

            gpio_set_level(arg.step_pin, 0);
        }
        else
            gpio_set_level(arg.step_pin, 1);
    }
    else
    {
        portENTER_CRITICAL(&spinlock);
        ESP_ERROR_CHECK(gptimer_stop(timer));
        portEXIT_CRITICAL(&spinlock);
    }

    return (high_task_awoken == pdTRUE);
}
#endif // MKS_STEP_MODE_ENABLE


/**
 * @brief initialize MKS UART and timers
 * 
 * @param mks_config struct with MKS connection parameters
 */
void mks_servo_init(mks_conf_t mks_config)
{
    // configure UART
    uart_config_t uart_config = {
        .baud_rate = mks_config.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
    };

    if (uart_is_driver_installed(mks_config.uart) == true)
        uart_driver_delete(mks_config.uart);

    ESP_ERROR_CHECK(uart_driver_install(mks_config.uart, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(mks_config.uart, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(mks_config.uart, mks_config.tx_pin, mks_config.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

#ifdef MKS_STEP_MODE_ENABLE
    uint8_t timer_N = mks_config.motor_num;

    portENTER_CRITICAL(&spinlock);
    gptimer = malloc(sizeof(gptimer_handle_t) * timer_N);
    cb_arg = malloc(sizeof(mks_cb_arg_t) * timer_N); // allocate memory for callback arguments
    portEXIT_CRITICAL(&spinlock);

    for (uint32_t i = 0; i < timer_N; i++)
    {
        // configure step and dir pins

        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
        io_conf.pin_bit_mask = ((1 << mks_config.step_pin[i]) | (1 << mks_config.dir_pin[i]) | (1 << mks_config.en_pin[i]));
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);

        mks_servo_enable(mks_config, i, false);
        mks_servo_set_dir(mks_config, i, MKS_CW_DIR);

        // configure timers

        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000 // 1 us
        };

        portENTER_CRITICAL(&spinlock);
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer[i]));
        portEXIT_CRITICAL(&spinlock);

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = 1000000, // 1 s
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true
        };

        portENTER_CRITICAL(&spinlock);
        ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer[i], &alarm_config));
        portEXIT_CRITICAL(&spinlock);

        gptimer_event_callbacks_t timer_cbs = {
            .on_alarm = mks_servo_clk_timer_callback
        };

        portENTER_CRITICAL(&spinlock);
        cb_arg[i].step_pin = mks_config.step_pin[i];
        cb_arg[i].motor_num = i;
        portEXIT_CRITICAL(&spinlock);

        portENTER_CRITICAL(&spinlock);
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer[i], &timer_cbs, (void*)&cb_arg[i]));
        ESP_ERROR_CHECK(gptimer_enable(gptimer[i]));
        portEXIT_CRITICAL(&spinlock);

        mks_servo_enable(mks_config, i, true); // enable motor

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

#endif // MKS_STEP_MODE_ENABLE
}


// deinit MKS UART and timers
void mks_servo_deinit(mks_conf_t mks_config)
{
#ifdef MKS_STEP_MODE_ENABLE
    for (uint32_t i = 0; i < mks_config.motor_num; i++)
    {
        portENTER_CRITICAL(&spinlock);
        ESP_ERROR_CHECK(gptimer_disable(gptimer[i]));
        ESP_ERROR_CHECK(gptimer_del_timer(gptimer[i]));
        portEXIT_CRITICAL(&spinlock);
    }

    portENTER_CRITICAL(&spinlock);
    free(gptimer);
    free(cb_arg);
    portEXIT_CRITICAL(&spinlock);
#endif // MKS_STEP_MODE_ENABLE

    if (uart_is_driver_installed(mks_config.uart) == true)
        ESP_ERROR_CHECK(uart_driver_delete(mks_config.uart));
}


// ============= STEP MODE FUNCTIONS =============


#ifdef MKS_STEP_MODE_ENABLE

/**
 * @brief set enable pin
 * 
 * @param mks_config struct with MKS connection parameters
 * @param motor_num motor number
 * @param enable false - disable (1), true - enable (0)
 */
void mks_servo_enable(mks_conf_t mks_config, uint8_t motor_num, bool enable)
{
    if (enable == false)
        gpio_set_level(mks_config.en_pin[motor_num], 1);
    else if (enable == true)
        gpio_set_level(mks_config.en_pin[motor_num], 0);
}


/**
 * @brief set direction pin
 * 
 * @param mks_config struct with MKS connection parameters
 * @param motor_num motor number
 * @param dir direction (CW_DIR or CCW_DIR)
 */
void mks_servo_set_dir(mks_conf_t mks_config, uint8_t motor_num, uint8_t dir)
{
    gpio_set_level(mks_config.dir_pin[motor_num], dir);
}


/**
 * @brief set period for step signal
 * 
 * @param motor_num motor number
 * @param period_us period in us
 */
void mks_servo_set_period(uint8_t motor_num, uint64_t period_us)
{
    period_us = period_us / 2; // 1 period = 2 gpio switches

    if (period_us == 0)
    {
        period_us = 1;
        ESP_LOGW(TAG, "Period equal 0 - set to 1 us!");
    }

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = period_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true
    };

    portENTER_CRITICAL(&spinlock);
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer[motor_num], &alarm_config));
    portEXIT_CRITICAL(&spinlock);
}


/**
 * @brief start/stop step signal
 * 
 * @param mks_config struct with MKS connection parameters
 * @param motor_num motor number
 * @param start false - stop, true - start
 */
void mks_servo_start(mks_conf_t mks_config, uint8_t motor_num, bool start)
{
    if (start == false)
    {
        portENTER_CRITICAL(&spinlock);
        ESP_ERROR_CHECK(gptimer_stop(gptimer[motor_num]));
        portEXIT_CRITICAL(&spinlock);
        gpio_set_level(mks_config.step_pin[motor_num], 0);
    }
    else if (start == true)
    {
        while (1) // wait for timer to stop
        {
            portENTER_CRITICAL(&spinlock);
            esp_err_t err = gptimer_start(gptimer[motor_num]);
            portEXIT_CRITICAL(&spinlock);

            if (err == ESP_OK)
                break;
            else
                ESP_LOGE(TAG, "waiting for timer %u to stop...", motor_num);

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}


/**
 * @brief move motor by desired number of steps with desired period and direction (sign in period_us variable) and with desired acceleration
 * 
 * @param mks_config struct with MKS connection parameters
 * @param steps steps to move
 * @param period_us period in us
 * @param accel_phase acceleration phase percentage (0.0f - 1.0f)
 */
void mks_servo_step_move(mks_conf_t mks_config, uint8_t motor_num, uint64_t steps, int64_t period_us, float accel_phase)
{
    if (steps != 0 && period_us != 0)
    {
        // set direction
        if (period_us < 0)
        {
            mks_servo_set_dir(mks_config, motor_num, MKS_CW_DIR);
            period_us = -period_us;
        }
        else
            mks_servo_set_dir(mks_config, motor_num, MKS_CCW_DIR);

        // acceleration parameters
        double s_0 = 0.0f;
        double dt = 0.0f;
        uint64_t period_us_cur = period_us;

        if (accel_phase > 0.0f && accel_phase < 1.0f)
        {
            double v_goal = 1.0f / (double)period_us;
            double time = (double)(steps * period_us);
            double t_0 = time * accel_phase;
            double accel = v_goal / t_0;
            s_0 = accel * t_0 * t_0 / 2.0f;
            dt = 2.0f * t_0 / s_0 / s_0;
            period_us_cur = (uint64_t)(s_0 * dt + (double)period_us);
        }

        portENTER_CRITICAL(&spinlock);
        cb_arg[motor_num].steps_left = steps;
        cb_arg[motor_num].steps_total = steps;
        cb_arg[motor_num].period_goal = period_us;

        cb_arg[motor_num].accel_s = s_0;

        cb_arg[motor_num].dt = dt;
        cb_arg[motor_num].time_passed = period_us_cur;
        portEXIT_CRITICAL(&spinlock);

        // set period
        mks_servo_set_period(motor_num, period_us_cur);

        // start timer and step signal
        mks_servo_start(mks_config, motor_num, true);
    }
    else
        ESP_LOGW(TAG, "Steps or period equal 0 - no move!");
}

#endif // MKS_STEP_MODE_ENABLE


// ============= UART MODE FUNCTIONS =============


/**
 * @brief read encoder value
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * 
 * @return encoder value
 */
float mks_servo_uart_read_encoder(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 10;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_ENCODER_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    int32_t carry = response[3] << 24 | response[4] << 16 | response[5] << 8 | response[6];
    uint16_t value = response[7] << 8 | response[8];

    float encoder_value = (float)carry * 360.0f + (float)value * 360.0f / (float)0x3FFF;

    return encoder_value;
}


/**
 * @brief read the number of pulses received.
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return number of pulses
 */
int32_t mks_servo_uart_read_pulses(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 8;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_PULSE_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    int32_t pulses = response[3] << 24 | response[4] << 16 | response[5] << 8 | response[6];

    return pulses;
}


/**
 * @brief read the error of the motor shaft angle
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * 
 * @return error of the motor shaft angle
 */
int16_t mks_servo_uart_read_motor_shaft_error(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 6;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_SHAFT_ERROR_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    int16_t shaft_error = response[3] << 8 | response[4];

    return shaft_error;
}


/**
 * @brief read enable status
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return enable status
 */
uint8_t mks_servo_uart_read_enable(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_ENABLE_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
}


/**
 * @brief read the go back to zero status when power on
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return go back to zero status (0 - going to zero, 1 - success, 2 - fail)
 */
uint8_t mks_servo_uart_read_go_to_zero_status(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_GO_TO_ZERO_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
}


/**
 * @brief release the motor shaft locked-rotor protection state
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_release_protection_state(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_RELEASE_SHAFT;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

#ifndef MKS_PC_RETURN
    mks_servo_uart_send(mks_config, datagram, len_w);
#endif

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief read the motor shaft protection state
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return motor shaft protection state (0 - not protected, 1 - protected)
 */
uint8_t mks_servo_uart_read_protection_state(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_SHAFT_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
}


/**
 * @brief calibrate the encoder
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 0 if calibrating, 1 if success, 2 if fail
 */
uint8_t mks_servo_uart_calibrate_encoder(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CAL_FUNC;
    datagram[3] = 0x00;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

#ifndef MKS_PC_RETURN
    mks_servo_uart_send(mks_config, datagram, len_w);
#endif

    // wait for 30 sec for encoder to calibrate
    vTaskDelay(30000 / portTICK_PERIOD_MS);

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);
    return response[3];
#endif

    return 1;
}


/**
 * @brief set the work mode
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param mode work mode (0 - CR_OPEN, 1 - CR_CLOSE, 2 - CR_vFOC, 3 - CR_UART)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_work_mode(mks_conf_t mks_config, uint8_t address, uint8_t mode)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_MODE_FUNC;
    datagram[3] = mode;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief set the current limit
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param ma current limit in mA (ma * 400 mA)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_current(mks_conf_t mks_config, uint8_t address, uint8_t ma)
{
    if (ma > 0x0D)
    {
        ESP_LOGW(TAG, "current limit is out of range");
        ma = 0x0D;
    }

    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_MA_FUNC;
    datagram[3] = ma;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief set subdivision
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param mstep microstep value
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_mstep(mks_conf_t mks_config, uint8_t address, uint8_t mstep)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_MSTEP_FUNC;
    datagram[3] = mstep;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief set the active of the enable pin
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param enable enable (0 - active low, 1 - active high, 2 - active always)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_enable(mks_conf_t mks_config, uint8_t address, uint8_t enable)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_EN_FUNC;
    datagram[3] = enable;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief set the direction of motor rotation
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param dir direction (0 - CW, 1 - CCW)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_dir(mks_conf_t mks_config, uint8_t address, uint8_t dir)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_DIR_FUNC;
    datagram[3] = dir;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief set the motor shaft locked-rotor protection function
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param enable enable (0 - disabled protection, 1 - enabled protection)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_shaft_protection(mks_conf_t mks_config, uint8_t address, uint8_t enable)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_PROTECT_FUNC;
    datagram[3] = enable;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief set the subdivision interpolation function
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param enable enable (0 - Mplyer disabled, 1 - Mplyer enabled)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_mplyer(mks_conf_t mks_config, uint8_t address, uint8_t enable)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_MPLYER_FUNC;
    datagram[3] = enable;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief set the baud rate
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param baud baud rate value
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_baud_rate(mks_conf_t mks_config, uint8_t address, uint8_t baud)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_UART_BAUD_FUNC;
    datagram[3] = baud;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief set the slave address
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param new_address new slave address
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_slave_address(mks_conf_t mks_config, uint8_t address, uint8_t new_address)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_UART_ADDR_FUNC;
    datagram[3] = new_address;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief set the slave respond
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param enable 1 - enable slave respond, 0 - disable slave respond
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_slave_respond(mks_conf_t mks_config, uint8_t address, uint8_t enable)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_UART_RSP_FUNC;
    datagram[3] = enable;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

#ifdef MKS_PC_RETURN
    if (enable == 1)
    {
        mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);
        return response[3];
    }
#endif

    return 1;
}


/**
 * @brief restore the default parameters
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_restore(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_RESTORE_FUNC;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

#ifndef MKS_PC_RETURN
    mks_servo_uart_send(mks_config, datagram, len_w);
#endif

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief query the motor status
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 0 - run fail, 1 - run starting, 2 - run complete
 */
uint8_t mks_servo_uart_cr_query(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_QUERY;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

#ifndef MKS_PC_RETURN
    mks_servo_uart_send(mks_config, datagram, len_w);
#endif

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief enable the motor
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param enable 1 - enable motor, 0 - disable motor
 * @return 0 - set fail, 1 - set success
 */
uint8_t mks_servo_uart_cr_enable(mks_conf_t mks_config, uint8_t address, uint8_t enable)
{
    if (enable != 1 && enable != 0)
    {
        enable = 0;
        ESP_LOGW(TAG, "Invalid enable status. Set to 0.");
    }

    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_ENABLE;
    datagram[3] = enable;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

#ifndef MKS_PC_RETURN
    mks_servo_uart_send(mks_config, datagram, len_w);
#endif

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief run the motor in speed mode
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param speed speed value with direction sign (-1279 - 1279)
 * @param accel acceleration value (0 - 32)
 * @return 0 - set fail, 1 - set success
 */
uint8_t mks_servo_uart_cr_run_w_speed(mks_conf_t mks_config, uint8_t address, int16_t speed, uint8_t accel)
{
    if (speed > MKS_MAX_SPEED)
    {
        ESP_LOGW(TAG, "speed is too high, set to %d", MKS_MAX_SPEED);
        speed = MKS_MAX_SPEED;
    }
    else if (speed < -MKS_MAX_SPEED)
    {
        ESP_LOGW(TAG, "speed is too high, set to %d", -MKS_MAX_SPEED);
        speed = -MKS_MAX_SPEED;
    }

    if (speed < 0)
    {
        speed = -speed;
        speed = speed | (1 << 15);
    }

    if (accel > 32)
    {
        ESP_LOGW(TAG, "accel is too high, set to 32");
        accel = 32;
    }

    uint8_t len_w = 7;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_MOTOR_TURN;
    datagram[3] = speed >> 8;
    datagram[4] = speed;
    datagram[5] = accel;
    datagram[6] = mks_servo_uart_calc_CRC(datagram, len_w);

#ifndef MKS_PC_RETURN
    mks_servo_uart_send(mks_config, datagram, len_w);
#endif

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);
    return response[3];
#endif

    return 1;
}


/**
 * @brief save the parameter in speed mode
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 0 - set fail, 1 - set success
 */
uint8_t mks_servo_uart_cr_save_params(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_SAVE_CLEAR;
    datagram[3] = MKS_CR_UART_STATUS_SAVE;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

#ifndef MKS_PC_RETURN
    mks_servo_uart_send(mks_config, datagram, len_w);
#endif

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief clear the parameter in speed mode
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 0 - set fail, 1 - set success
 */
uint8_t mks_servo_uart_cr_clear_params(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_SAVE_CLEAR;
    datagram[3] = MKS_CR_UART_STATUS_CLEAR;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

#ifndef MKS_PC_RETURN
    mks_servo_uart_send(mks_config, datagram, len_w);
#endif

#ifdef MKS_PC_RETURN
    mks_servo_uart_send_w_recv_check(mks_config, address, datagram, len_w, response, len_r);

    return response[3];
#endif

    return 1;
}


/**
 * @brief run the motor in position mode
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param speed speed value with direction sign (-1279 - 1279)
 * @param accel acceleration value (0 - 32)
 * @param pulses number of pulses
 * @return 0 - set fail, 1 - set success
 */
uint8_t mks_servo_uart_cr_set_pos(mks_conf_t mks_config, uint8_t address, int16_t speed, uint8_t accel, uint32_t pulses)
{
    if (speed > MKS_MAX_SPEED)
    {
        ESP_LOGW(TAG, "speed is too high, set to %d", MKS_MAX_SPEED);
        speed = MKS_MAX_SPEED;
    }
    else if (speed < -MKS_MAX_SPEED)
    {
        ESP_LOGW(TAG, "speed is too high, set to %d", -MKS_MAX_SPEED);
        speed = -MKS_MAX_SPEED;
    }

    if (speed < 0)
    {
        speed = -speed;
        speed = speed | (1 << 15);
    }

    if (accel > 32)
    {
        ESP_LOGW(TAG, "accel is too high, set to 32");
        accel = 32;
    }

    uint8_t len_w = 11;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

#ifdef MKS_PC_RETURN
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;
#endif

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_MOTOR_RUN;
    datagram[3] = (speed >> 8) & 0xFF;
    datagram[4] = speed & 0xFF;
    datagram[5] = accel;
    datagram[6] = (pulses >> 24) & 0xFF;
    datagram[7] = (pulses >> 16) & 0xFF;
    datagram[8] = (pulses >> 8) & 0xFF;
    datagram[9] = pulses & 0xFF;
    datagram[10] = mks_servo_uart_calc_CRC(datagram, len_w);

#ifndef MKS_PC_RETURN
    mks_servo_uart_send(mks_config, datagram, len_w);
#endif

#ifdef MKS_PC_RETURN
    // wait until motor stops
    uint32_t buf = 0;

    while (1)
    {
        buf = uart_read_bytes(mks_config.uart, response, len_r, portMAX_DELAY);
        uart_flush(mks_config.uart);

        if (buf == len_r)
        {
            if (response[0] == MKS_DOWNLINK_HEAD && response[1] == address && response[2] == MKS_CR_UART_MOTOR_RUN)
            {
                if (response[3] == 2)
                    return response[3];
                else if (response[3] == 1)
                    continue;
                else
                    ESP_LOGE(TAG, "UART read error - move not ended");
            }
        }
        else
            ESP_LOGE(TAG, "UART read error");
    }

    return 0;
#endif

    return 1;
}