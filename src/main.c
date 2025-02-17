#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include <ctype.h> // Required for isdigit()

#define LOG_LEVEL LOG_LEVEL_DBG
#define RX_BUF_SIZE 10
#define RX_TIMEOUT 100

LOG_MODULE_REGISTER(main, LOG_LEVEL);

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart20));

// Create LED array
static const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios)
};

#define NUM_LEDS ARRAY_SIZE(leds)

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

static uint8_t rx_buf[RX_BUF_SIZE] = {0};

static uint8_t tx_buf[] =   {"Hello from Civan's Project\r\n"
    "Press 1-4 on your keyboard to toggle LEDS 1-4 on your development kit\r\n"};

static int initialize_uart(void) {
    int ret;

    if (!device_is_ready(uart)) {
        LOG_ERR("UART device not ready");
        return -ENODEV; // Keep specific error code for device not ready
    }
    LOG_INF("UART device is ready");

    ret = uart_callback_set(uart, uart_cb, NULL); // Set UART callback
    if (ret) {
        LOG_ERR("Failed to set UART callback: %d", ret);
        return 1; // Generic error for callback setup failure
    }
    LOG_INF("UART callback set successfully");
    return 0; // Success
}

static int initialize_leds(void) {
    int ret;

    for (int i = 0; i < NUM_LEDS; i++) {
        if (!device_is_ready(leds[i].port)) {
            LOG_ERR("LED%d device not ready", i);
            return 1; // Generic error for LED device not ready (port)
        }
        LOG_INF("LED%d device is ready", i);

        ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure LED%d pin: %d", i, ret);
            return 1; // Generic error for pin configuration failure
        }
        LOG_INF("LED%d pin configured", i);
    }
    return 0; // Success
}


static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
    switch (evt->type) {
        case UART_RX_RDY:
            LOG_DBG("Received %d bytes", evt->data.rx.len);
            if (evt->data.rx.len == 1) {
                char received_char = evt->data.rx.buf[evt->data.rx.offset];
                if (isdigit(received_char)) { // Check if received char is a digit
                    int led_index = received_char - '111'; // Convert char to integer index
                    if (led_index >= 0 && led_index < NUM_LEDS) { // Check if index is within LED array bounds
                        LOG_DBG("Received '%c', toggling LED%d", received_char, led_index);
                        gpio_pin_toggle_dt(&leds[led_index]);
                    } else {
                        LOG_WRN("Received digit '%c' out of LED range (0-%d)", received_char, NUM_LEDS - 1);
                    }
                } else {
                    LOG_WRN("Received non-digit character '%c', expecting digits '0' to '%d' for LED control",
                            received_char, NUM_LEDS - 1);
                }
            }
            break;
        case UART_RX_DISABLED:
            LOG_DBG("RX disabled, re-enabling");
            uart_rx_enable(dev, rx_buf, sizeof(rx_buf), RX_TIMEOUT);
            break;

        default:
            break;
    }
}

int main(void) {
    int ret;

    ret = initialize_uart();
    if (ret) {
        return ret; // Propagate error from UART initialization
    }

    ret = initialize_leds();
    if (ret) {
        return ret; // Propagate error from LED initialization
    }

    // ... rest of your main application logic (if any) ...
    LOG_INF("Initialization complete, application started");


    ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US); // Send initial message
    if (ret) {
        LOG_ERR("Failed to send initial message: %d", ret);
        return 1; // Generic error for initial message send failure
    }
    // blink leds to indicate initialization complete
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_pin_toggle_dt(&leds[i]);
        k_sleep(K_MSEC(100));
    }

    uart_rx_enable(uart, rx_buf, sizeof(rx_buf), RX_TIMEOUT); // Enable UART RX

    while (1) {
        k_sleep(K_FOREVER); // Sleep indefinitely
    }

    return 0; // Indicate successful overall initialization and application start
}