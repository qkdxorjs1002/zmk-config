#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_connection_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/sleep.h>
#include <zmk/keymap.h>
#include <zmk/ble.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_NODELABEL(status_led), gpios);
static struct k_work_delayable blink_off_work;
static struct k_work_delayable adv_blink_work;
static struct k_work_delayable connected_blink_work;
static bool adv_blink_state = false;
static bool connected_blinking = false;
static bool suspended = false;

static void led_set(bool on) {
    if (!device_is_ready(led.port)) return;
    gpio_pin_set_dt(&led, on ? 0 : 1);
}

static void blink_once(int on_ms, int off_ms) {
    led_set(true);
    k_msleep(on_ms);
    led_set(false);
    k_msleep(off_ms);
}

static void blink_n_times(int count, int on_ms, int off_ms) {
    for (int i = 0; i < count; i++) {
        blink_once(on_ms, off_ms);
    }
}

static void adv_blink(struct k_work *work) {
    if (suspended) return;
    adv_blink_state = !adv_blink_state;
    led_set(adv_blink_state);
    k_work_schedule(&adv_blink_work, K_MSEC(300));
}

static void connected_blink(struct k_work *work) {
    if (suspended) return;
    led_set(true);
    k_msleep(50);
    led_set(false);
    k_work_schedule(&connected_blink_work, K_MSEC(1000));
}

static int status_led_listener(const struct zmk_event_header *eh) {
    if (is_zmk_ble_connection_state_changed(eh)) {
        const struct zmk_ble_connection_state_changed *ev = cast_zmk_ble_connection_state_changed(eh);

        if (ev->connected) {
            k_work_cancel_delayable(&adv_blink_work);
            connected_blinking = true;
            k_work_schedule(&connected_blink_work, K_MSEC(1000));
        } else {
            connected_blinking = false;
            k_work_cancel_delayable(&connected_blink_work);
            k_work_schedule(&adv_blink_work, K_MSEC(300));
        }

    } else if (is_zmk_ble_active_profile_changed(eh)) {
        uint8_t profile = zmk_ble_active_profile_index() + 1;
        blink_n_times(profile, 100, 100);

    } else if (is_zmk_layer_state_changed(eh)) {
        uint8_t layer = zmk_keymap_highest_layer_active() + 1;
        blink_n_times(layer, 80, 80);

    } else if (is_zmk_sleep(eh)) {
        const struct zmk_sleep *ev = cast_zmk_sleep(eh);
        if (ev->state == ZMK_SLEEP_STATE_SUSPEND) {
            suspended = true;
            k_work_cancel_delayable(&adv_blink_work);
            k_work_cancel_delayable(&connected_blink_work);
            led_set(false);
        } else if (ev->state == ZMK_SLEEP_STATE_RESUME) {
            suspended = false;
            if (connected_blinking) {
                k_work_schedule(&connected_blink_work, K_MSEC(1000));
            } else {
                k_work_schedule(&adv_blink_work, K_MSEC(300));
            }
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(status_led, status_led_listener);
ZMK_SUBSCRIPTION(status_led, zmk_ble_connection_state_changed);
ZMK_SUBSCRIPTION(status_led, zmk_ble_active_profile_changed);
ZMK_SUBSCRIPTION(status_led, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(status_led, zmk_sleep);

static int status_led_init(void) {
    if (!device_is_ready(led.port)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    k_work_init_delayable(&adv_blink_work, adv_blink);
    k_work_init_delayable(&connected_blink_work, connected_blink);

    k_work_schedule(&adv_blink_work, K_MSEC(300));

    return 0;
}

SYS_INIT(status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
