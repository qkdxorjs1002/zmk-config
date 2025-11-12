#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#if IS_ENABLED(CONFIG_ZMK_BLE)
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/ble.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_KEYMAP)
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_NODELABEL(status_led), gpios);

static struct k_work_delayable adv_blink_work;
static struct k_work_delayable conn_blink_tick_work;
static struct k_work_delayable conn_blink_off_work;
static struct k_work_delayable state_eval_work;
static struct k_work_delayable seq_blink_work;

static bool is_connected = false;
static bool suspended = false;
static bool seq_running = false;
static int seq_remaining = 0;
static int seq_on_ms = 100, seq_off_ms = 100;

static inline void led_set(bool on) {
    if (!device_is_ready(led.port)) return;
    gpio_pin_set_dt(&led, on ? 0 : 1);
}

static void adv_blink_fn(struct k_work *work) {
    if (suspended || seq_running || is_connected) return;
    static bool adv_on = false;
    adv_on = !adv_on;
    led_set(adv_on);
    k_work_schedule(&adv_blink_work, K_MSEC(300));
}

static void conn_blink_off_fn(struct k_work *work) { led_set(false); }

static void conn_blink_tick_fn(struct k_work *work) {
    if (suspended || seq_running || !is_connected) return;
    led_set(true);
    k_work_schedule(&conn_blink_off_work, K_MSEC(50));
    k_work_schedule(&conn_blink_tick_work, K_MSEC(1000));
}

static void seq_blink_fn(struct k_work *work) {
    if (suspended) { seq_running = false; led_set(false); return; }
    if (seq_remaining <= 0) {
        seq_running = false;
        led_set(false);
        if (is_connected) k_work_schedule(&conn_blink_tick_work, K_MSEC(1000));
        else k_work_schedule(&adv_blink_work, K_MSEC(300));
        return;
    }
    led_set(true);
    k_work_schedule(&conn_blink_off_work, K_MSEC(seq_on_ms));
    seq_remaining--;
    k_work_schedule(&seq_blink_work, K_MSEC(seq_on_ms + seq_off_ms));
}

static void seq_start(int count, int on_ms, int off_ms) {
    if (count <= 0) return;
    seq_running = true;
    seq_remaining = count;
    seq_on_ms = on_ms;
    seq_off_ms = off_ms;
    k_work_cancel_delayable(&adv_blink_work);
    k_work_cancel_delayable(&conn_blink_tick_work);
    k_work_schedule(&seq_blink_work, K_NO_WAIT);
}

static void state_eval_fn(struct k_work *work) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    bool now = zmk_ble_active_profile_is_connected();
#else
    bool now = false;
#endif
    if (!suspended && now != is_connected && !seq_running) {
        is_connected = now;
        led_set(false);
        k_work_cancel_delayable(&adv_blink_work);
        k_work_cancel_delayable(&conn_blink_tick_work);
        if (is_connected) k_work_schedule(&conn_blink_tick_work, K_MSEC(1000));
        else k_work_schedule(&adv_blink_work, K_MSEC(300));
    }
    k_work_schedule(&state_eval_work, K_MSEC(250));
}

static int status_led_listener(const zmk_event_t *eh) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    const struct zmk_ble_active_profile_changed *ap = as_zmk_ble_active_profile_changed(eh);
    if (ap) {
        int n = zmk_ble_active_profile_index() + 1;
        seq_start(n, 120, 120);
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_KEYMAP)
    const struct zmk_layer_state_changed *ls = as_zmk_layer_state_changed(eh);
    if (ls) {
        int n = zmk_keymap_highest_layer_active() + 1;
        seq_start(n, 90, 90);
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    const struct zmk_activity_state_changed *ac = as_zmk_activity_state_changed(eh);
    if (ac) {
        if (ac->state == ZMK_ACTIVITY_SLEEP) {
            suspended = true;
            k_work_cancel_delayable(&adv_blink_work);
            k_work_cancel_delayable(&conn_blink_tick_work);
            k_work_cancel_delayable(&state_eval_work);
            k_work_cancel_delayable(&seq_blink_work);
            led_set(false);
        } else if (ac->state == ZMK_ACTIVITY_ACTIVE) {
            suspended = false;
            k_work_schedule(&state_eval_work, K_NO_WAIT);
            if (!seq_running) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
                is_connected = zmk_ble_active_profile_is_connected();
#else
                is_connected = false;
#endif
                if (is_connected) k_work_schedule(&conn_blink_tick_work, K_MSEC(1000));
                else k_work_schedule(&adv_blink_work, K_MSEC(300));
            }
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(status_led, status_led_listener);

#if IS_ENABLED(CONFIG_ZMK_BLE)
ZMK_SUBSCRIPTION(status_led, zmk_ble_active_profile_changed);
#endif

#if IS_ENABLED(CONFIG_ZMK_KEYMAP)
ZMK_SUBSCRIPTION(status_led, zmk_layer_state_changed);
#endif

ZMK_SUBSCRIPTION(status_led, zmk_activity_state_changed);

static int status_led_init(void) {
    if (!device_is_ready(led.port)) return -ENODEV;
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    k_work_init_delayable(&adv_blink_work, adv_blink_fn);
    k_work_init_delayable(&conn_blink_tick_work, conn_blink_tick_fn);
    k_work_init_delayable(&conn_blink_off_work, conn_blink_off_fn);
    k_work_init_delayable(&state_eval_work, state_eval_fn);
    k_work_init_delayable(&seq_blink_work, seq_blink_fn);
    suspended = false;
    k_work_schedule(&state_eval_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
