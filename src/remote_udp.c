// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/uart.h"

// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"

// Include custom libraries
#include "vga16_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_3.h"
#include "mfe_algorithm.h"
#include "imu_container.h"
#include <string.h>
#include <stdlib.h>
#include "stdio.h"

// Include udp libraries
#include "lwip/pbuf.h"
#include "lwip/udp.h"
// #include "lwip/opt.h"
// #include "lwip/debug.h"
// #include "lwip/stats.h"
// #include "lwip/dns.h"
// #include "lwip/netif.h"

// =======================================
// udp constants
#define UDP_PORT 1234
#define UDP_MSG_LEN_MAX 128
#define udp_target_pico "10.49.244.179"
// =======================================
// WIFI 
#define WIFI_SSID "RedRover"
#define WIFI_PASSWORD NULL
// =======================================

// SEND DATA
fix15 send_data[UDP_MSG_LEN_MAX];
struct pt_sem new_udp_send_s;

// data to send over WIFI
#define max_data_size  128
int data_size = 7;
fix15 data_array[max_data_size];

// the following MUST be less than or equal to:
// UDP_MSG_LEN_MAX
// but for efficiency, not much smaller
#define send_data_size data_size*sizeof(fix15)
#define max_send_data_size max_data_size*sizeof(fix15)

// Global variables
#define BUTTON_PIN 21
#define CONFIRM_BUTTON_PIN 6

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV  25.0
uint slice_num ;

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// Interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // mpu6050_read_raw(acceleration, gyro);
    
    // data_array[0] = (fix15) acceleration[0];
    // data_array[1] = (fix15) acceleration[1];
    // data_array[2] = (fix15) acceleration[2];
    // data_array[3] = (fix15) gyro[0];         
    // data_array[4] = (fix15) gyro[1];
    // data_array[5] = (fix15) gyro[2];    
    // data_array[6] = (fix15) 0; // Placeholder for additional data
}

void udp_sender() {
    struct udp_pcb* pcb = udp_new();

    pcb->remote_port = UDP_PORT ;
    pcb->local_port = UDP_PORT ;

    static ip_addr_t addr;
    ipaddr_aton(udp_target_pico, &addr);

    bool button_pressed_last = false; 
    int counter = 0;
    uint64_t start_swing_time; // = (time_us_64()/1000);

    bool confirm_button_pressed_last = false;
    uint32_t last_button_press_time = 0;
    const uint32_t DEBOUNCE_DELAY = 400; // Debounce delay in milliseconds


    while (1) {
        bool button_pressed = !gpio_get(BUTTON_PIN);
        bool confirm_button_pressed = !gpio_get(CONFIRM_BUTTON_PIN);
        uint32_t current_time = time_us_64() / 1000;

        //add conditional statements 
        if (confirm_button_pressed && !confirm_button_pressed_last && 
            (current_time - last_button_press_time > DEBOUNCE_DELAY)){

            last_button_press_time = current_time;
            
            mpu6050_read_raw(acceleration, gyro);
            printf("%f %f %f %f %f %f\n", fix2float15(acceleration[0]), fix2float15(acceleration[1]), fix2float15(acceleration[2]), fix2float15(gyro[0]), fix2float15(gyro[1]), fix2float15(gyro[2]));

            data_array[0] = int2fix15(-2);
            data_array[1] = int2fix15(-2);
            data_array[2] = int2fix15(-2);
            data_array[3] = int2fix15(-2);
            data_array[4] = int2fix15(-2);
            data_array[5] = int2fix15(-2);

            uint64_t time_us = (time_us_64()/1000) - start_swing_time;
            data_array[6] = int2fix15(time_us);
            printf("Time: %lld ms\n", time_us);

            printf("Confirmation Button PRESSED! \n");

            struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, send_data_size+1, PBUF_RAM);
            char *req = (char *)p->payload;

            memset(send_data, 0, max_send_data_size);
            memcpy(send_data, data_array, send_data_size) ;

            //clear the payload of buffer 
            memset(req, 0, send_data_size+1);

            //copy send data in payload? 
            memcpy(req, send_data, send_data_size) ;

            //send the packet 
            err_t er = udp_sendto(pcb, p, &addr, UDP_PORT); //port

            pbuf_free(p);

            //Errors
            if (er != ERR_OK) {
                printf("Failed to send UDP packet! error=%d\n", er);
            }else {
                printf("Sent packet %d\n", counter);
                counter++;
            }

            while(!gpio_get(CONFIRM_BUTTON_PIN)){
                sleep_ms(10);
            }

        }

        if (button_pressed && !button_pressed_last) {
            printf("Button PRESSED!\n");
            start_swing_time = (time_us_64()/1000);
        }

        if (button_pressed) { 
            // actual data-send            
            mpu6050_read_raw(acceleration, gyro);

            printf("%f %f %f %f %f %f\n", fix2float15(acceleration[0]), fix2float15(acceleration[1]), fix2float15(acceleration[2]), fix2float15(gyro[0]), fix2float15(gyro[1]), fix2float15(gyro[2]));
            data_array[0] = acceleration[0];
            data_array[1] = acceleration[1];
            data_array[2] = acceleration[2];
            data_array[3] = gyro[0];
            data_array[4] = gyro[1];
            data_array[5] = gyro[2];

            uint64_t time_us = (time_us_64()/1000) - start_swing_time;
            data_array[6] = int2fix15(time_us);
            printf("Time: %lld ms\n", time_us);

            struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, send_data_size+1, PBUF_RAM);
            char *req = (char *)p->payload;
            memset(send_data, 0, max_send_data_size);
            memcpy(send_data, data_array, send_data_size) ;

            //clear the payload of buffer 
            memset(req, 0, send_data_size+1);

            //copy send data in payload? 
            memcpy(req, send_data, send_data_size) ;

            //send the packet 
            err_t er = udp_sendto(pcb, p, &addr, UDP_PORT); //port

            pbuf_free(p);

            //Errors
            if (er != ERR_OK) {
                printf("Failed to send UDP packet! error=%d\n", er);
            }else {
                printf("Sent packet %d\n", counter);
                counter++;
            }
        }
        
        if (! button_pressed && button_pressed_last) {
            data_array[0] = int2fix15(-1); // Signal end of data
            data_array[1] = int2fix15(-1); // Signal end of data
            data_array[2] = int2fix15(-1); // Signal end of data
            data_array[3] = int2fix15(-1); // Signal end of data
            data_array[4] = int2fix15(-1); // Signal end of data
            data_array[5] = int2fix15(-1); // Signal end of data
            data_array[6] = int2fix15(-1); // Signal end of data
            printf("Button RELEASED! \n");

            struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, send_data_size+1, PBUF_RAM);
            char *req = (char *)p->payload;

            memset(send_data, 0, max_send_data_size);
            memcpy(send_data, data_array, send_data_size) ;

            //clear the payload of buffer 
            memset(req, 0, send_data_size+1);

            //copy send data in payload? 
            memcpy(req, send_data, send_data_size) ;

            //send the packet 
            err_t er = udp_sendto(pcb, p, &addr, UDP_PORT); //port

            pbuf_free(p);
            //Errors? 
            if (er != ERR_OK) {
                printf("Failed to send UDP packet! error=%d\n", er);
            }else {
                printf("Sent packet %d\n", counter);
                counter++;
            }
        }

        button_pressed_last = button_pressed;
        confirm_button_pressed_last = confirm_button_pressed;
        // sleep_ms(10);
    }
}

int main() {
    // Initialize stdio
    stdio_init_all();
    printf("Running remote_udp\n");

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// Button CONFIGURATION /////////////////////////
    ////////////////////////////////////////////////////////////////////////
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    gpio_init(CONFIRM_BUTTON_PIN);
    gpio_set_dir(CONFIRM_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(CONFIRM_BUTTON_PIN);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM

    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)

    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler

    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals

    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle

    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel

    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    /////////////////////// Init CYW43 architecture ////////////////////////
    ////////////////////////////////////////////////////////////////////////
    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        printf("IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    }
    
    // Start UDP remote logic
    udp_sender();

    // struct udp_pc
    cyw43_arch_deinit();
    return 0;
}