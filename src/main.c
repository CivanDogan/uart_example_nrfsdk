#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <ctype.h> // Required for isdigit()

// Logging
#define LOG_MODULE_NAME main_app
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

// Configuration Macros
#define UART_DEVICE_NODE DT_NODELABEL(uart20)
#define RX_BUFFER_SIZE    10
#define RX_TIMEOUT_MS     100
#define MESSAGE_BLINK_DELAY_MS 100

// LED Configuration (Using array for multiple LEDs)
static const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios)
};
#define NUM_LEDS ARRAY_SIZE(leds)

// UART Device Instance
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

// RX Buffer
static uint8_t rx_buffer[RX_BUFFER_SIZE];

// TX Message (moved to constant for clarity)
static const uint8_t tx_message[] =
    "Hello from Civan's Project\r\n"
    "Press 1-4 to toggle LEDs 1-4\r\n";


// Function Prototypes (forward declarations for better organization)
static int     init_uart(void);
static int     init_leds(void);
static void    uart_receive_callback(const struct device *dev, struct uart_event *evt, void *user_data);
static void    process_uart_data(char received_char);
static void    blink_leds(uint32_t blink_delay_ms);


/**
 * @brief UART Receive Callback Function
 *
 * This function is called by the UART driver when UART events occur,
 * in this case, handling received data.
 *
 * @param dev      Pointer to the UART device structure.
 * @param evt      Pointer to the UART event structure containing event information.
 * @param user_data User-defined data (not used in this example).
 */
static void uart_receive_callback(const struct device *dev, struct uart_event *evt, void *user_data) {
    switch (evt->type) {
        case UART_RX_RDY:
            if (evt->data.rx.len == 1) {
                process_uart_data(evt->data.rx.buf[evt->data.rx.offset]);
            } else {
                LOG_WRN("Received unexpected number of bytes: %d", evt->data.rx.len);
            }
            break;

        case UART_RX_DISABLED:
            LOG_DBG("RX disabled, re-enabling");
            uart_rx_enable(dev, rx_buffer, sizeof(rx_buffer), RX_TIMEOUT_MS);
            break;

        case UART_RX_BUF_REQUEST:
        case UART_RX_BUF_RELEASED:
        case UART_TX_DONE:
        case UART_TX_ABORTED:
        default:
            LOG_DBG("UART event: %d", evt->type); // Log other events for debugging if needed
            break;
    }
}

/**
 * @brief Processes received UART data.
 *
 * Checks if the received character is a digit and toggles the corresponding LED.
 * Logs warnings for invalid input.
 *
 * @param received_char The character received from UART.
 */
static void process_uart_data(char received_char) {
    if (isdigit(received_char)) {
        int led_index = received_char - '1'; // Convert char '1'-'4' to index 0-3
        if (led_index >= 0 && led_index < NUM_LEDS) {
            LOG_DBG("Toggling LED %d", led_index + 1);
            gpio_pin_toggle_dt(&leds[led_index]);
        } else {
            LOG_WRN("Received digit '%c' out of LED range (1-%d)", received_char, NUM_LEDS);
        }
    } else {
        LOG_WRN("Received non-digit '%c', expecting digits '1' to '%d'",
                received_char, NUM_LEDS);
    }
}


/**
 * @brief Initializes the UART device and sets the receive callback.
 *
 * @return 0 on success, negative error code on failure.
 */
static int init_uart(void) {
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device %s not ready", uart_dev->name);
        return -ENODEV;
    }
    LOG_INF("UART device %s is ready", uart_dev->name);

    int ret = uart_callback_set(uart_dev, uart_receive_callback, NULL);
    if (ret) {
        LOG_ERR("Failed to set UART callback (%d)", ret);
        return ret;
    }
    LOG_INF("UART callback set");
    return 0;
}

/**
 * @brief Initializes the LEDs specified in the device tree.
 *
 * Configures each LED as a GPIO output and sets initial state (ACTIVE).
 *
 * @return 0 on success, negative error code on failure.
 */
static int init_leds(void) {
    int ret;
    for (int i = 0; i < NUM_LEDS; i++) {
        if (!device_is_ready(leds[i].port)) {
            LOG_ERR("LED%d port not ready", i);
            return -ENODEV;
        }
        LOG_INF("LED%d port is ready", i);

        ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure LED%d pin (%d)", i, ret);
            return ret;
        }
        LOG_INF("LED%d pin configured", i);
    }
    return 0;
}

/**
 * @brief Blinks all LEDs a few times to indicate initialization complete.
 *
 * @param blink_delay_ms Delay between toggles in milliseconds.
 */
static void blink_leds(uint32_t blink_delay_ms) {
    LOG_INF("Blinking LEDs to indicate initialization");
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_pin_toggle_dt(&leds[i]);
        k_msleep(blink_delay_ms);
    }
    for (int i = 0; i < NUM_LEDS; i++) { // Toggle back to initial state (all off if ACTIVE is high)
        gpio_pin_toggle_dt(&leds[i]);
    }
}


/**
 * @brief Main application entry point.
 *
 * Initializes UART and LEDs, sends initial message, enables UART RX,
 * and then enters a forever loop.
 *
 * @return Always returns 0 (or exits if initialization fails).
 */
int main(void) {
    int ret;

    LOG_INF("Starting application");

    ret = init_uart();
    if (ret) {
        LOG_ERR("UART initialization failed (%d)", ret);
        return ret;
    }

    ret = init_leds();
    if (ret) {
        LOG_ERR("LED initialization failed (%d)", ret);
        return ret;
    }

    blink_leds(MESSAGE_BLINK_DELAY_MS);
    LOG_INF("Initialization complete");


    ret = uart_tx(uart_dev, tx_message, sizeof(tx_message), SYS_FOREVER_US);
    if (ret) {
        LOG_ERR("Failed to send initial message (%d)", ret);
        return ret;
    }
    LOG_INF("Initial message sent");


    ret = uart_rx_enable(uart_dev, rx_buffer, sizeof(rx_buffer), RX_TIMEOUT_MS);
    if (ret) {
        LOG_ERR("Failed to enable UART RX (%d)", ret);
        return ret;
    }
    LOG_INF("UART RX enabled");


    while (1) {
        k_sleep(K_FOREVER); // Idle loop
    }

    return 0;
}