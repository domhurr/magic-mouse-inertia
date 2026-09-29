#define _GNU_SOURCE

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <libevdev/libevdev-uinput.h>
#include <libevdev/libevdev.h>
#include <linux/input.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define NS_PER_MS 1000000LL
#define NS_PER_SECOND 1000000000LL
#define HI_RES_UNITS_PER_STEP 120
#define TICK_MS 8
#define MAX_TOUCH_SLOTS 32
#define POINTER_MOTION_WINDOW_MS 110
#define POINTER_MOTION_THRESHOLD 24
#define SUSPECT_VERTICAL_REPORTS 2
#define VERTICAL_DOMINANCE_RATIO 1.5
#define SUSTAINED_VERTICAL_INTERVALS 4
#define FRONT_ANCHOR_MAX_Y -600
#define REAR_CONTACT_MIN_Y FRONT_ANCHOR_MAX_Y
#define SIDE_CONTACT_ABS_X 1200
#define RAW_SCROLL_COHERENT_MS 150
#define RAW_SCROLL_COHERENT_DISTANCE 90
#define RAW_SCROLL_AXIS_NONE 0
#define RAW_SCROLL_AXIS_X 1
#define RAW_SCROLL_AXIS_Y 2

struct config {
    const char *device_name;
    double idle_ms;
    double decay_ms;
    double gain;
    double min_velocity;
    double max_velocity;
    double max_duration_ms;
    double touch_scale;
    double min_flick_distance;
    bool verbose;
};

struct touch_slot_state {
    bool active;
    bool began_after_pointer_motion;
    bool have_x;
    bool have_y;
    bool have_touch_major;
    bool have_touch_minor;
    bool have_sample;
    bool dirty;
    int x;
    int y;
    int last_x;
    int last_y;
    int first_x;
    int first_y;
    int touch_major;
    int touch_minor;
    int max_touch_major;
    int max_touch_minor;
    int report_delta_x;
    int report_delta_y;
    bool have_report_delta;
    int coherent_x_direction;
    int coherent_x_distance;
    int64_t coherent_x_started_ns;
    int coherent_y_direction;
    int coherent_y_distance;
    int64_t coherent_y_started_ns;
    bool raw_scroll_armed;
    int raw_scroll_axis;
    unsigned int motion_intervals_x;
    unsigned int motion_intervals_y;
    int64_t last_sample_ns;
    double velocity_x;
    double velocity_y;
};

struct axis_state {
    double velocity;
    double output_fraction;
    int low_res_remainder;
};

struct daemon_state {
    struct axis_state x;
    struct axis_state y;
    int pending_x_hi;
    int pending_y_hi;
    int pending_x_low;
    int pending_y_low;
    bool saw_x_hi;
    bool saw_y_hi;
    struct touch_slot_state touch_slots[MAX_TOUCH_SLOTS];
    int current_touch_slot;
    int touch_count;
    int peak_touch_count;
    bool have_contact_start;
    int64_t contact_started_ns;
    bool touch_release_observed;
    bool have_touch_release;
    double touch_release_x;
    double touch_release_y;
    bool touch_release_sustained_vertical;
    bool touch_release_pointer_suspect;
    bool touch_release_scroll_armed;
    bool have_scroll_time;
    double scroll_abs_x;
    double scroll_abs_y;
    double scroll_sum_x;
    double scroll_sum_y;
    unsigned int vertical_scroll_reports;
    bool two_front_contacts_seen;
    int raw_scroll_slot;
    double generated_scroll_x;
    double generated_scroll_y;
    double generated_fraction_x;
    double generated_fraction_y;
    int pointer_motion_amount;
    int64_t last_pointer_motion_ns;
    bool inertia_active;
    int64_t last_scroll_ns;
    int64_t inertia_started_ns;
    int64_t last_tick_ns;
};

static volatile sig_atomic_t running = 1;

static void handle_signal(int signo)
{
    (void)signo;
    running = 0;
}

static int64_t monotonic_ns(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
    }

    return (int64_t)ts.tv_sec * NS_PER_SECOND + ts.tv_nsec;
}

static double clamp_double(double value, double minimum, double maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static void reset_inertia(struct daemon_state *state)
{
    state->inertia_active = false;
    state->x.output_fraction = 0.0;
    state->y.output_fraction = 0.0;
    state->x.low_res_remainder = 0;
    state->y.low_res_remainder = 0;
}

static void reset_gesture(struct daemon_state *state)
{
    reset_inertia(state);
    state->have_scroll_time = false;
    state->x.velocity = 0.0;
    state->y.velocity = 0.0;
    state->pending_x_hi = 0;
    state->pending_y_hi = 0;
    state->pending_x_low = 0;
    state->pending_y_low = 0;
    state->saw_x_hi = false;
    state->saw_y_hi = false;
    state->peak_touch_count = state->touch_count;
    state->have_contact_start = false;
    state->touch_release_observed = false;
    state->have_touch_release = false;
    state->touch_release_x = 0.0;
    state->touch_release_y = 0.0;
    state->touch_release_sustained_vertical = false;
    state->touch_release_pointer_suspect = false;
    state->touch_release_scroll_armed = false;
    state->scroll_abs_x = 0.0;
    state->scroll_abs_y = 0.0;
    state->scroll_sum_x = 0.0;
    state->scroll_sum_y = 0.0;
    state->vertical_scroll_reports = 0;
    state->two_front_contacts_seen = false;
    state->raw_scroll_slot = -1;
}

static void reset_touch_state(struct daemon_state *state)
{
    memset(state->touch_slots, 0, sizeof(state->touch_slots));
    state->current_touch_slot = 0;
    state->touch_count = 0;
}

static void reset_input_state(struct daemon_state *state)
{
    reset_gesture(state);
    reset_touch_state(state);
    state->generated_scroll_x = 0.0;
    state->generated_scroll_y = 0.0;
    state->generated_fraction_x = 0.0;
    state->generated_fraction_y = 0.0;
}

static void rebuild_touch_state(struct daemon_state *state,
                                const struct libevdev *device)
{
    int slot_count = libevdev_get_num_slots(device);
    int current_slot = libevdev_get_current_slot(device);
    bool have_x = libevdev_has_event_code(
        device, EV_ABS, ABS_MT_POSITION_X);
    bool have_y = libevdev_has_event_code(
        device, EV_ABS, ABS_MT_POSITION_Y);
    bool have_touch_major = libevdev_has_event_code(
        device, EV_ABS, ABS_MT_TOUCH_MAJOR);
    bool have_touch_minor = libevdev_has_event_code(
        device, EV_ABS, ABS_MT_TOUCH_MINOR);
    int64_t now = monotonic_ns();
    int slot;

    reset_touch_state(state);
    if (slot_count < 0)
        return;
    if (slot_count > MAX_TOUCH_SLOTS)
        slot_count = MAX_TOUCH_SLOTS;
    if (current_slot >= 0 && current_slot < slot_count)
        state->current_touch_slot = current_slot;

    for (slot = 0; slot < slot_count; slot++) {
        struct touch_slot_state *touch = &state->touch_slots[slot];

        if (libevdev_get_slot_value(device, slot,
                                    ABS_MT_TRACKING_ID) < 0)
            continue;

        touch->active = true;
        touch->have_x = have_x;
        touch->have_y = have_y;
        if (have_x)
            touch->x = libevdev_get_slot_value(
                device, slot, ABS_MT_POSITION_X);
        if (have_y)
            touch->y = libevdev_get_slot_value(
                device, slot, ABS_MT_POSITION_Y);
        touch->first_x = touch->last_x = touch->x;
        touch->first_y = touch->last_y = touch->y;
        touch->have_sample = have_x && have_y;
        touch->last_sample_ns = now;

        touch->have_touch_major = have_touch_major;
        if (have_touch_major) {
            touch->touch_major = libevdev_get_slot_value(
                device, slot, ABS_MT_TOUCH_MAJOR);
            touch->max_touch_major = touch->touch_major;
        }
        touch->have_touch_minor = have_touch_minor;
        if (have_touch_minor) {
            touch->touch_minor = libevdev_get_slot_value(
                device, slot, ABS_MT_TOUCH_MINOR);
            touch->max_touch_minor = touch->touch_minor;
        }
        state->touch_count++;
    }
    state->peak_touch_count = state->touch_count;
}

static bool touch_state_matches_device(const struct daemon_state *state,
                                       const struct libevdev *device)
{
    int slot_count = libevdev_get_num_slots(device);
    int device_touch_count = 0;
    int slot;

    if (slot_count < 0)
        return state->touch_count == 0;

    for (slot = 0; slot < slot_count; slot++) {
        bool active = libevdev_get_slot_value(
                          device, slot, ABS_MT_TRACKING_ID) >= 0;

        if (slot < MAX_TOUCH_SLOTS) {
            if (state->touch_slots[slot].active != active)
                return false;
            if (active)
                device_touch_count++;
        } else if (active) {
            return false;
        }
    }
    return state->touch_count == device_touch_count;
}

static void reconcile_touch_state(struct daemon_state *state,
                                  const struct libevdev *device,
                                  const struct config *config)
{
    int previous_count;

    if (touch_state_matches_device(state, device))
        return;

    previous_count = state->touch_count;
    reset_input_state(state);
    rebuild_touch_state(state, device);
    if (config->verbose)
        fprintf(stderr,
                "touch state repaired: tracked=%d kernel=%d\n",
                previous_count, state->touch_count);
}

static int resync_device(struct daemon_state *state,
                         struct libevdev *device,
                         const struct config *config)
{
    struct input_event event;
    int previous_count = state->touch_count;
    int error;

    reset_input_state(state);
    while ((error = libevdev_next_event(
                device, LIBEVDEV_READ_FLAG_SYNC, &event)) ==
           LIBEVDEV_READ_STATUS_SYNC)
        ;
    if (error != -EAGAIN)
        return error;

    rebuild_touch_state(state, device);
    if (config->verbose)
        fprintf(stderr,
                "input resynchronized after dropped events: "
                "tracked=%d kernel=%d\n",
                previous_count, state->touch_count);
    return 0;
}

static void update_axis_velocity(double *velocity, double instantaneous)
{
    const double smoothing = 0.45;

    if (*velocity == 0.0 || (*velocity > 0.0) != (instantaneous > 0.0))
        *velocity = instantaneous;
    else
        *velocity = (1.0 - smoothing) * *velocity + smoothing * instantaneous;
}

static void update_touch_sample(struct touch_slot_state *slot, int64_t now)
{
    if (!slot->dirty || !slot->have_x || !slot->have_y)
        return;

    if (slot->have_sample) {
        int delta_x = slot->x - slot->last_x;
        int delta_y = slot->y - slot->last_y;
        double elapsed =
            (double)(now - slot->last_sample_ns) / NS_PER_SECOND;

        if (abs(delta_x) >= 8)
            slot->motion_intervals_x++;
        if (abs(delta_y) >= 8)
            slot->motion_intervals_y++;
        slot->report_delta_x = delta_x;
        slot->report_delta_y = delta_y;
        slot->have_report_delta = true;

        if (abs(delta_x) >= 8) {
            int direction = delta_x > 0 ? 1 : -1;

            if (slot->coherent_x_direction != direction ||
                slot->coherent_x_started_ns == 0 ||
                now - slot->coherent_x_started_ns >
                    RAW_SCROLL_COHERENT_MS * NS_PER_MS) {
                slot->coherent_x_direction = direction;
                slot->coherent_x_distance = abs(delta_x);
                slot->coherent_x_started_ns = now;
            } else {
                slot->coherent_x_distance += abs(delta_x);
            }
        }

        if (abs(delta_y) >= 8) {
            int direction = delta_y > 0 ? 1 : -1;

            if (slot->coherent_y_direction != direction ||
                slot->coherent_y_started_ns == 0 ||
                now - slot->coherent_y_started_ns >
                    RAW_SCROLL_COHERENT_MS * NS_PER_MS) {
                slot->coherent_y_direction = direction;
                slot->coherent_y_distance = abs(delta_y);
                slot->coherent_y_started_ns = now;
            } else {
                slot->coherent_y_distance += abs(delta_y);
            }
        }

        if (elapsed >= 0.001 && elapsed <= 0.080) {
            update_axis_velocity(&slot->velocity_x,
                                 delta_x / elapsed);
            update_axis_velocity(&slot->velocity_y,
                                 delta_y / elapsed);
        } else {
            slot->velocity_x = 0.0;
            slot->velocity_y = 0.0;
        }
    }

    if (!slot->have_sample) {
        slot->first_x = slot->x;
        slot->first_y = slot->y;
    }
    slot->last_x = slot->x;
    slot->last_y = slot->y;
    slot->last_sample_ns = now;
    slot->have_sample = true;
    slot->dirty = false;
}

static void finish_touch_report(struct daemon_state *state, int64_t now)
{
    int slot;

    for (slot = 0; slot < MAX_TOUCH_SLOTS; slot++) {
        if (state->touch_slots[slot].active)
            update_touch_sample(&state->touch_slots[slot], now);
    }
}

static bool is_front_contact(const struct touch_slot_state *slot)
{
    return slot->active && slot->have_sample &&
           slot->last_y <= FRONT_ANCHOR_MAX_Y;
}

static int count_front_contacts(const struct daemon_state *state)
{
    int count = 0;
    int slot;

    for (slot = 0; slot < MAX_TOUCH_SLOTS; slot++) {
        if (is_front_contact(&state->touch_slots[slot]))
            count++;
    }

    return count;
}

static bool rear_contact_is_blocked(const struct daemon_state *state,
                                    int candidate_slot)
{
    int slot;

    if (state->touch_slots[candidate_slot].last_y <
        REAR_CONTACT_MIN_Y)
        return false;

    for (slot = 0; slot < MAX_TOUCH_SLOTS; slot++) {
        if (slot != candidate_slot &&
            is_front_contact(&state->touch_slots[slot]))
            return true;
    }

    return false;
}

static void finish_raw_scroll_report(struct daemon_state *state,
                                     const struct config *config)
{
    int selected = state->raw_scroll_slot;
    int front_contacts = count_front_contacts(state);
    int slot;

    if (front_contacts >= 2)
        state->two_front_contacts_seen = true;

    if (selected >= 0 &&
        selected < MAX_TOUCH_SLOTS &&
        !state->touch_slots[selected].active &&
        !state->touch_slots[selected].have_report_delta)
        selected = -1;

    if (selected < 0) {
        double best_displacement = 0.0;
        int best_axis = RAW_SCROLL_AXIS_NONE;

        for (slot = 0; slot < MAX_TOUCH_SLOTS; slot++) {
            struct touch_slot_state *touch =
                &state->touch_slots[slot];
            int displacement_x;
            int displacement_y;
            double horizontal;
            double vertical;
            double candidate_displacement;
            int candidate_axis;

            if (!touch->have_sample ||
                (!touch->active && !touch->have_report_delta))
                continue;

            displacement_x = touch->last_x - touch->first_x;
            displacement_y = touch->last_y - touch->first_y;
            horizontal = fabs((double)displacement_x);
            vertical = fabs((double)displacement_y);

            candidate_axis = RAW_SCROLL_AXIS_NONE;
            candidate_displacement = 0.0;
            if (horizontal >= config->min_flick_distance &&
                horizontal >= vertical * VERTICAL_DOMINANCE_RATIO &&
                touch->coherent_x_distance >=
                    RAW_SCROLL_COHERENT_DISTANCE) {
                candidate_axis = RAW_SCROLL_AXIS_X;
                candidate_displacement = horizontal;
            } else if (vertical >= config->min_flick_distance &&
                       vertical >= horizontal *
                                       VERTICAL_DOMINANCE_RATIO &&
                       touch->coherent_y_distance >=
                           RAW_SCROLL_COHERENT_DISTANCE) {
                candidate_axis = RAW_SCROLL_AXIS_Y;
                candidate_displacement = vertical;
            }

            if (candidate_axis == RAW_SCROLL_AXIS_NONE ||
                abs(touch->last_x) >= SIDE_CONTACT_ABS_X ||
                rear_contact_is_blocked(state, slot))
                continue;

            if (candidate_displacement > best_displacement) {
                best_displacement = candidate_displacement;
                best_axis = candidate_axis;
                selected = slot;
            }
        }

        if (selected >= 0) {
            state->raw_scroll_slot = selected;
            state->touch_slots[selected].raw_scroll_armed = true;
            state->touch_slots[selected].raw_scroll_axis = best_axis;
            if (best_axis == RAW_SCROLL_AXIS_X) {
                state->scroll_abs_x += best_displacement;
                state->scroll_sum_x +=
                    (state->touch_slots[selected].last_x -
                     state->touch_slots[selected].first_x) *
                    config->touch_scale;
            } else {
                state->scroll_abs_y += best_displacement;
                state->scroll_sum_y +=
                    -(state->touch_slots[selected].last_y -
                      state->touch_slots[selected].first_y) *
                    config->touch_scale;
                state->vertical_scroll_reports++;
            }
            reset_inertia(state);
            if (config->verbose)
                fprintf(stderr,
                        "raw scroll armed: slot=%d axis=%c "
                        "anchors protect y>=%d\n",
                        selected,
                        best_axis == RAW_SCROLL_AXIS_X ? 'x' : 'y',
                        REAR_CONTACT_MIN_Y);
        }
    }

    if (selected >= 0 && selected < MAX_TOUCH_SLOTS) {
        struct touch_slot_state *touch =
            &state->touch_slots[selected];

        if (!touch->raw_scroll_armed &&
            rear_contact_is_blocked(state, selected)) {
            if (config->verbose)
                fprintf(stderr,
                        "raw scroll blocked: rear slot=%d y=%d\n",
                        selected, touch->last_y);
            state->raw_scroll_slot = -1;
        } else if (touch->have_report_delta) {
            double amount;

            if (touch->raw_scroll_axis == RAW_SCROLL_AXIS_X) {
                amount = touch->report_delta_x *
                         config->touch_scale;
                state->generated_scroll_x += amount;
                state->scroll_abs_x += fabs(amount);
                state->scroll_sum_x += amount;
            } else {
                amount = -touch->report_delta_y *
                         config->touch_scale;
                state->generated_scroll_y += amount;
                state->scroll_abs_y += fabs(amount);
                state->scroll_sum_y += amount;
                state->vertical_scroll_reports++;
            }
            state->last_scroll_ns = monotonic_ns();
            state->have_scroll_time = true;
            reset_inertia(state);
        }

        if (!touch->active)
            state->raw_scroll_slot = -1;
    }

    for (slot = 0; slot < MAX_TOUCH_SLOTS; slot++)
        state->touch_slots[slot].have_report_delta = false;
}

static void finish_physical_report(struct daemon_state *state,
                                   const struct config *config,
                                   int64_t now)
{
    int delta_x = state->saw_x_hi ? state->pending_x_hi
                                  : state->pending_x_low * HI_RES_UNITS_PER_STEP;
    int delta_y = state->saw_y_hi ? state->pending_y_hi
                                  : state->pending_y_low * HI_RES_UNITS_PER_STEP;

    state->pending_x_hi = 0;
    state->pending_y_hi = 0;
    state->pending_x_low = 0;
    state->pending_y_low = 0;
    state->saw_x_hi = false;
    state->saw_y_hi = false;

    if (delta_x == 0 && delta_y == 0)
        return;

    if (state->inertia_active && config->verbose)
        fprintf(stderr, "new physical scroll: cancelling inertia\n");
    reset_inertia(state);

    if (!state->have_scroll_time) {
        state->scroll_abs_x = 0.0;
        state->scroll_abs_y = 0.0;
        state->scroll_sum_x = 0.0;
        state->scroll_sum_y = 0.0;
    }
    state->scroll_abs_x += fabs((double)delta_x);
    state->scroll_abs_y += fabs((double)delta_y);
    state->scroll_sum_x += delta_x;
    state->scroll_sum_y += delta_y;
    if (delta_y != 0)
        state->vertical_scroll_reports++;

    if (state->have_scroll_time) {
        double elapsed = (double)(now - state->last_scroll_ns) / NS_PER_SECOND;

        if (elapsed > 0.001 && elapsed < 0.080) {
            if (delta_x != 0)
                update_axis_velocity(&state->x.velocity, delta_x / elapsed);
            else
                state->x.velocity *= 0.70;

            if (delta_y != 0)
                update_axis_velocity(&state->y.velocity, delta_y / elapsed);
            else
                state->y.velocity *= 0.70;
        } else {
            state->x.velocity = 0.0;
            state->y.velocity = 0.0;
        }
    } else if (state->have_contact_start) {
        double elapsed =
            (double)(now - state->contact_started_ns) / NS_PER_SECOND;

        /*
         * The kernel's scroll threshold can collapse a short flick into one
         * high-resolution report. Cap the contact interval so that this first
         * report still carries a useful release-velocity estimate.
         */
        elapsed = clamp_double(elapsed, 0.008, 0.080);
        state->x.velocity = delta_x / elapsed;
        state->y.velocity = delta_y / elapsed;
    }

    state->last_scroll_ns = now;
    state->have_scroll_time = true;
}

static int write_axis_events(struct libevdev_uinput *output,
                             unsigned int low_code,
                             unsigned int high_code,
                             struct axis_state *axis,
                             int high_delta)
{
    int low_delta = 0;
    int error;

    if (high_delta == 0)
        return 0;

    axis->low_res_remainder += high_delta;
    while (axis->low_res_remainder >= HI_RES_UNITS_PER_STEP) {
        axis->low_res_remainder -= HI_RES_UNITS_PER_STEP;
        low_delta++;
    }
    while (axis->low_res_remainder <= -HI_RES_UNITS_PER_STEP) {
        axis->low_res_remainder += HI_RES_UNITS_PER_STEP;
        low_delta--;
    }

    if (low_delta != 0) {
        error = libevdev_uinput_write_event(output, EV_REL, low_code, low_delta);
        if (error != 0)
            return error;
    }

    return libevdev_uinput_write_event(output, EV_REL, high_code, high_delta);
}

static int emit_generated_scroll(struct libevdev_uinput *output,
                                 struct daemon_state *state)
{
    double x_amount =
        state->generated_scroll_x + state->generated_fraction_x;
    double y_amount =
        state->generated_scroll_y + state->generated_fraction_y;
    int x_delta = (int)trunc(x_amount);
    int y_delta = (int)trunc(y_amount);
    int error;

    state->generated_scroll_x = 0.0;
    state->generated_scroll_y = 0.0;
    state->generated_fraction_x = x_amount - x_delta;
    state->generated_fraction_y = y_amount - y_delta;

    if (x_delta == 0 && y_delta == 0)
        return 0;

    error = write_axis_events(output, REL_HWHEEL,
                              REL_HWHEEL_HI_RES, &state->x,
                              x_delta);
    if (error != 0)
        return error;
    error = write_axis_events(output, REL_WHEEL,
                              REL_WHEEL_HI_RES, &state->y,
                              y_delta);
    if (error != 0)
        return error;

    return libevdev_uinput_write_event(output, EV_SYN, SYN_REPORT, 0);
}

static int emit_inertia_tick(struct libevdev_uinput *output,
                             struct daemon_state *state,
                             const struct config *config,
                             int64_t now)
{
    double elapsed = (double)(now - state->last_tick_ns) / NS_PER_SECOND;
    double decay = exp(-elapsed / (config->decay_ms / 1000.0));
    double x_amount;
    double y_amount;
    int x_delta;
    int y_delta;
    int error;

    elapsed = clamp_double(elapsed, 0.001, 0.050);
    state->x.velocity *= decay;
    state->y.velocity *= decay;

    x_amount = state->x.velocity * elapsed + state->x.output_fraction;
    y_amount = state->y.velocity * elapsed + state->y.output_fraction;
    x_delta = (int)trunc(x_amount);
    y_delta = (int)trunc(y_amount);
    state->x.output_fraction = x_amount - x_delta;
    state->y.output_fraction = y_amount - y_delta;

    error = write_axis_events(output, REL_HWHEEL, REL_HWHEEL_HI_RES,
                              &state->x, x_delta);
    if (error != 0)
        return error;

    error = write_axis_events(output, REL_WHEEL, REL_WHEEL_HI_RES,
                              &state->y, y_delta);
    if (error != 0)
        return error;

    if (x_delta != 0 || y_delta != 0) {
        error = libevdev_uinput_write_event(output, EV_SYN, SYN_REPORT, 0);
        if (error != 0)
            return error;
    }

    state->last_tick_ns = now;
    return 0;
}

static bool should_stop_inertia(const struct daemon_state *state,
                                const struct config *config,
                                int64_t now)
{
    double age_ms = (double)(now - state->inertia_started_ns) / NS_PER_MS;

    return age_ms >= config->max_duration_ms ||
           (fabs(state->x.velocity) < config->min_velocity * 0.18 &&
            fabs(state->y.velocity) < config->min_velocity * 0.18);
}

static void maybe_start_inertia(struct daemon_state *state,
                                const struct config *config,
                                int64_t now)
{
    double idle_ms;
    double speed;
    double scale;

    if (state->inertia_active)
        return;

    if (state->two_front_contacts_seen &&
        !state->touch_release_scroll_armed &&
        (state->touch_release_observed ||
         state->have_touch_release)) {
        if (config->verbose)
            fprintf(stderr,
                    "inertia rejected: two front button contacts\n");
        state->have_scroll_time = false;
        state->have_contact_start = false;
        state->touch_release_observed = false;
        state->have_touch_release = false;
        state->x.velocity = 0.0;
        state->y.velocity = 0.0;
        return;
    }

    /*
     * A touch release is the authoritative end of a Magic Mouse gesture.
     * If its raw motion did not qualify as a flick, do not let an older
     * wheel estimate manufacture inertia for an ordinary finger lift.
     */
    if (state->touch_release_observed &&
        !state->have_touch_release) {
        if (config->verbose)
            fprintf(stderr,
                    "inertia rejected: touch release was not a flick\n");
        state->have_scroll_time = false;
        state->have_contact_start = false;
        state->touch_release_observed = false;
        state->x.velocity = 0.0;
        state->y.velocity = 0.0;
        return;
    }

    if (!state->have_scroll_time && !state->have_touch_release)
        return;

    if (state->have_touch_release) {
        /*
         * Prefer the release sample outright. Wheel velocity can lag behind
         * the finger and otherwise turn a slow release into a false flick.
         */
        state->x.velocity = state->touch_release_x;
        state->y.velocity = state->touch_release_y;

        /*
         * Base scrolling is generated only after one coherent, dominant raw
         * axis has passed the palm filters. Keep release inertia on that same
         * axis instead of blending perpendicular contact jitter into it.
         */
        if (state->scroll_abs_x > 0.0 ||
            state->scroll_abs_y > 0.0) {
            if (state->scroll_abs_y >=
                state->scroll_abs_x * 1.35) {
                state->x.velocity = 0.0;
            } else if (state->scroll_abs_x >=
                       state->scroll_abs_y * 1.35) {
                state->y.velocity = 0.0;
            }
        }
        if (state->scroll_abs_x > 0.0 &&
            state->x.velocity * state->scroll_sum_x < 0.0)
            state->x.velocity = 0.0;
        if (state->scroll_abs_y > 0.0 &&
            state->y.velocity * state->scroll_sum_y < 0.0)
            state->y.velocity = 0.0;

        if (state->peak_touch_count > 1 &&
            state->scroll_abs_x == 0.0 &&
            state->scroll_abs_y == 0.0) {
            if (config->verbose)
                fprintf(stderr,
                        "inertia rejected: multi-touch release had no "
                        "physical scroll intent\n");
            state->have_scroll_time = false;
            state->have_contact_start = false;
            state->touch_release_observed = false;
            state->have_touch_release = false;
            state->x.velocity = 0.0;
            state->y.velocity = 0.0;
            return;
        }

        /*
         * A touch arriving just after brisk chassis motion is commonly a
         * palm brushing the shell. It may use raw-touch inertia only after
         * demonstrating real vertical scroll intent; pointer motion during
         * an already-active touch never contaminates the gesture.
         */
        if (state->touch_release_pointer_suspect) {
            state->x.velocity = 0.0;
            if (state->vertical_scroll_reports <
                    SUSPECT_VERTICAL_REPORTS &&
                !state->touch_release_sustained_vertical) {
                if (config->verbose)
                    fprintf(stderr,
                            "inertia rejected: touch followed pointer "
                            "motion without proven vertical intent "
                            "(reports=%u sustained=%d)\n",
                            state->vertical_scroll_reports,
                            state->touch_release_sustained_vertical);
                state->have_scroll_time = false;
                state->have_contact_start = false;
                state->touch_release_observed = false;
                state->have_touch_release = false;
                state->x.velocity = 0.0;
                state->y.velocity = 0.0;
                return;
            }
        }
    } else if (state->touch_count == 0) {
        idle_ms = (double)(now - state->last_scroll_ns) / NS_PER_MS;
        if (idle_ms < config->idle_ms)
            return;
    } else {
        return;
    }

    speed = hypot(state->x.velocity, state->y.velocity);
    if (speed < config->min_velocity) {
        if (config->verbose)
            fprintf(stderr,
                    "inertia rejected: speed=%.1f below minimum=%.1f\n",
                    speed, config->min_velocity);
        state->have_scroll_time = false;
        state->have_contact_start = false;
        state->touch_release_observed = false;
        state->have_touch_release = false;
        return;
    }

    state->x.velocity *= config->gain;
    state->y.velocity *= config->gain;
    speed = hypot(state->x.velocity, state->y.velocity);
    if (speed > config->max_velocity) {
        scale = config->max_velocity / speed;
        state->x.velocity *= scale;
        state->y.velocity *= scale;
    }

    state->inertia_active = true;
    state->have_scroll_time = false;
    state->have_contact_start = false;
    state->touch_release_observed = false;
    state->have_touch_release = false;
    state->inertia_started_ns = now;
    state->last_tick_ns = now;
    state->x.output_fraction = 0.0;
    state->y.output_fraction = 0.0;
    state->x.low_res_remainder = 0;
    state->y.low_res_remainder = 0;

    if (config->verbose)
        fprintf(stderr, "inertia start: vx=%.1f vy=%.1f units/s\n",
                state->x.velocity, state->y.velocity);
}

static int find_device(const char *wanted_name, char *path, size_t path_size)
{
    DIR *directory;
    struct dirent *entry;
    int result = -ENOENT;

    directory = opendir("/dev/input");
    if (directory == NULL)
        return -errno;

    while ((entry = readdir(directory)) != NULL) {
        struct libevdev *device = NULL;
        char candidate[256];
        const char *name;
        int fd;
        int error;

        if (strncmp(entry->d_name, "event", 5) != 0)
            continue;

        if (snprintf(candidate, sizeof(candidate), "/dev/input/%s",
                     entry->d_name) >= (int)sizeof(candidate))
            continue;

        fd = open(candidate, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            if (errno == EACCES)
                result = -EACCES;
            continue;
        }

        error = libevdev_new_from_fd(fd, &device);
        if (error == 0) {
            name = libevdev_get_name(device);
            if (name != NULL && strcmp(name, wanted_name) == 0 &&
                libevdev_has_event_code(device, EV_ABS,
                                        ABS_MT_POSITION_X) &&
                libevdev_has_event_code(device, EV_ABS,
                                        ABS_MT_POSITION_Y)) {
                if (strlen(candidate) + 1 > path_size) {
                    result = -ENAMETOOLONG;
                } else {
                    strcpy(path, candidate);
                    result = 0;
                }
            }
            libevdev_free(device);
        }
        close(fd);

        if (result == 0)
            break;
    }

    closedir(directory);
    return result;
}

static int create_output(struct libevdev_uinput **output)
{
    struct libevdev *template = libevdev_new();
    int error;

    if (template == NULL)
        return -ENOMEM;

    libevdev_set_name(template, "Magic Mouse Inertia");
    libevdev_set_id_bustype(template, BUS_VIRTUAL);
    libevdev_set_id_vendor(template, 0x1);
    libevdev_set_id_product(template, 0x1);
    libevdev_set_id_version(template, 1);
    libevdev_enable_event_type(template, EV_KEY);
    libevdev_enable_event_code(template, EV_KEY, BTN_LEFT, NULL);
    libevdev_enable_event_code(template, EV_KEY, BTN_RIGHT, NULL);
    libevdev_enable_event_code(template, EV_KEY, BTN_MIDDLE, NULL);
    libevdev_enable_event_type(template, EV_REL);
    libevdev_enable_event_code(template, EV_REL, REL_X, NULL);
    libevdev_enable_event_code(template, EV_REL, REL_Y, NULL);
    libevdev_enable_event_code(template, EV_REL, REL_WHEEL, NULL);
    libevdev_enable_event_code(template, EV_REL, REL_HWHEEL, NULL);
    libevdev_enable_event_code(template, EV_REL, REL_WHEEL_HI_RES, NULL);
    libevdev_enable_event_code(template, EV_REL, REL_HWHEEL_HI_RES, NULL);

    error = libevdev_uinput_create_from_device(
        template, LIBEVDEV_UINPUT_OPEN_MANAGED, output);
    libevdev_free(template);
    return error;
}

static int calculate_poll_timeout(const struct daemon_state *state,
                                  const struct config *config,
                                  int64_t now)
{
    int64_t deadline;
    int64_t remaining_ns;

    if (state->inertia_active)
        deadline = state->last_tick_ns + TICK_MS * NS_PER_MS;
    else if (state->have_touch_release)
        return 0;
    else if (state->have_scroll_time)
        deadline = state->last_scroll_ns +
                   (int64_t)(config->idle_ms * NS_PER_MS);
    else
        return 1000;

    remaining_ns = deadline - now;
    if (remaining_ns <= 0)
        return 0;

    return (int)((remaining_ns + NS_PER_MS - 1) / NS_PER_MS);
}

static void process_event(struct daemon_state *state,
                          const struct config *config,
                          const struct input_event *event)
{
    if (event->type == EV_REL) {
        switch (event->code) {
        case REL_HWHEEL_HI_RES:
            state->pending_x_hi += event->value;
            state->saw_x_hi = true;
            reset_inertia(state);
            break;
        case REL_WHEEL_HI_RES:
            state->pending_y_hi += event->value;
            state->saw_y_hi = true;
            reset_inertia(state);
            break;
        case REL_HWHEEL:
            state->pending_x_low += event->value;
            reset_inertia(state);
            break;
        case REL_WHEEL:
            state->pending_y_low += event->value;
            reset_inertia(state);
            break;
        case REL_X:
        case REL_Y:
            if (state->touch_count == 0) {
                int64_t now = monotonic_ns();

                if (state->last_pointer_motion_ns == 0 ||
                    now - state->last_pointer_motion_ns >
                        POINTER_MOTION_WINDOW_MS * NS_PER_MS)
                    state->pointer_motion_amount = 0;
                state->pointer_motion_amount += abs(event->value);
                if (state->pointer_motion_amount > 100000)
                    state->pointer_motion_amount = 100000;
                state->last_pointer_motion_ns = now;
            }
            break;
        default:
            break;
        }
    } else if (event->type == EV_ABS && event->code == ABS_MT_SLOT) {
        if (event->value >= 0 && event->value < MAX_TOUCH_SLOTS)
            state->current_touch_slot = event->value;
    } else if (event->type == EV_ABS &&
               (event->code == ABS_MT_POSITION_X ||
                event->code == ABS_MT_POSITION_Y ||
                event->code == ABS_MT_TOUCH_MAJOR ||
                event->code == ABS_MT_TOUCH_MINOR)) {
        int slot = state->current_touch_slot;

        if (slot >= 0 && slot < MAX_TOUCH_SLOTS) {
            if (event->code == ABS_MT_POSITION_X) {
                state->touch_slots[slot].x = event->value;
                state->touch_slots[slot].have_x = true;
            } else if (event->code == ABS_MT_POSITION_Y) {
                state->touch_slots[slot].y = event->value;
                state->touch_slots[slot].have_y = true;
            } else if (event->code == ABS_MT_TOUCH_MAJOR) {
                state->touch_slots[slot].touch_major = event->value;
                state->touch_slots[slot].have_touch_major = true;
                if (event->value >
                    state->touch_slots[slot].max_touch_major)
                    state->touch_slots[slot].max_touch_major =
                        event->value;
            } else {
                state->touch_slots[slot].touch_minor = event->value;
                state->touch_slots[slot].have_touch_minor = true;
                if (event->value >
                    state->touch_slots[slot].max_touch_minor)
                    state->touch_slots[slot].max_touch_minor =
                        event->value;
            }
            if (event->code == ABS_MT_POSITION_X ||
                event->code == ABS_MT_POSITION_Y)
                state->touch_slots[slot].dirty = true;
        }
    } else if (event->type == EV_ABS &&
               event->code == ABS_MT_TRACKING_ID) {
        int slot = state->current_touch_slot;

        if (slot >= 0 && slot < MAX_TOUCH_SLOTS) {
            if (event->value >= 0 &&
                !state->touch_slots[slot].active) {
                bool first_contact = state->touch_count == 0;
                bool cancelling_inertia = state->inertia_active;
                int64_t now = monotonic_ns();
                bool follows_pointer_motion =
                    first_contact &&
                    state->last_pointer_motion_ns != 0 &&
                    now - state->last_pointer_motion_ns <=
                        POINTER_MOTION_WINDOW_MS * NS_PER_MS &&
                    state->pointer_motion_amount >=
                        POINTER_MOTION_THRESHOLD;

                if (first_contact || cancelling_inertia)
                    reset_gesture(state);
                if (first_contact) {
                    state->have_contact_start = true;
                    state->contact_started_ns = now;
                    state->pointer_motion_amount = 0;
                    state->last_pointer_motion_ns = 0;
                }
                memset(&state->touch_slots[slot], 0,
                       sizeof(state->touch_slots[slot]));
                state->touch_slots[slot].active = true;
                state->touch_slots[slot].began_after_pointer_motion =
                    follows_pointer_motion;
                state->touch_count++;
                if (state->touch_count > state->peak_touch_count)
                    state->peak_touch_count = state->touch_count;
                if (config->verbose)
                    fprintf(stderr,
                            "finger contact: slot=%d count=%d%s%s\n",
                            slot, state->touch_count,
                            cancelling_inertia
                                ? " cancelling inertia"
                                : "",
                            follows_pointer_motion
                                ? " after pointer motion"
                                : "");
            } else if (event->value < 0 &&
                       state->touch_slots[slot].active) {
                struct touch_slot_state *touch =
                    &state->touch_slots[slot];
                int64_t now = monotonic_ns();
                double sample_age;
                int displacement_x;
                int displacement_y;
                bool qualifies_x;
                bool qualifies_y;
                bool horizontal_dominant;
                bool vertical_dominant;
                bool qualified_release = false;

                update_touch_sample(touch, now);
                sample_age =
                    (double)(now - touch->last_sample_ns) / NS_PER_MS;
                displacement_x = touch->last_x - touch->first_x;
                displacement_y = touch->last_y - touch->first_y;
                horizontal_dominant =
                    fabs((double)displacement_x) >=
                    fabs((double)displacement_y) *
                        VERTICAL_DOMINANCE_RATIO;
                vertical_dominant =
                    fabs((double)displacement_y) >=
                    fabs((double)displacement_x) *
                        VERTICAL_DOMINANCE_RATIO;
                qualifies_x =
                    abs(displacement_x) >= config->min_flick_distance &&
                    touch->motion_intervals_x >= 2 &&
                    horizontal_dominant;
                qualifies_y =
                    abs(displacement_y) >= config->min_flick_distance &&
                    touch->motion_intervals_y >= 2 &&
                    vertical_dominant;
                if (touch->raw_scroll_armed) {
                    if (touch->raw_scroll_axis == RAW_SCROLL_AXIS_X)
                        qualifies_y = false;
                    else if (touch->raw_scroll_axis ==
                             RAW_SCROLL_AXIS_Y)
                        qualifies_x = false;
                }

                if (touch->have_sample && sample_age <= 80.0 &&
                    (qualifies_x || qualifies_y)) {
                    double candidate_x =
                        qualifies_x
                            ? touch->velocity_x * config->touch_scale
                            : 0.0;
                    double candidate_y =
                        qualifies_y
                            ? -touch->velocity_y * config->touch_scale
                            : 0.0;

                    /*
                     * hid_magicmouse emits horizontal wheel movement with
                     * touch X, and vertical wheel movement opposite touch Y.
                     */
                    if (!state->have_touch_release ||
                        hypot(candidate_x, candidate_y) >
                            hypot(state->touch_release_x,
                                  state->touch_release_y)) {
                        state->touch_release_x = candidate_x;
                        state->touch_release_y = candidate_y;
                        state->touch_release_sustained_vertical =
                            qualifies_y &&
                            fabs((double)displacement_y) >=
                                config->min_flick_distance * 2.0 &&
                            touch->motion_intervals_y >=
                                SUSTAINED_VERTICAL_INTERVALS;
                        state->touch_release_pointer_suspect =
                            touch->began_after_pointer_motion;
                        state->touch_release_scroll_armed =
                            touch->raw_scroll_armed;
                        state->have_touch_release = true;
                        qualified_release = true;
                    }
                } else if (config->verbose) {
                    fprintf(stderr,
                            "touch release rejected: slot=%d "
                            "have_sample=%d sample_age=%.1fms "
                            "displacement=(%d,%d) intervals=(%u,%u)\n",
                            slot, touch->have_sample, sample_age,
                            displacement_x, displacement_y,
                            touch->motion_intervals_x,
                            touch->motion_intervals_y);
                }
                touch->active = false;
                if (state->touch_count > 0)
                    state->touch_count--;
                /*
                 * An unqualified secondary contact is palm noise, not the
                 * end of the scrolling gesture. A qualified moving contact
                 * or the final contact release remains authoritative.
                 */
                if (qualified_release || state->touch_count == 0)
                    state->touch_release_observed = true;
                if (config->verbose)
                    fprintf(stderr,
                            "finger release: slot=%d count=%d "
                            "raw-v=(%.1f,%.1f) scroll-v=(%.1f,%.1f) "
                            "pos=(%d,%d)->(%d,%d) area=%dx%d max=%dx%d\n",
                            slot, state->touch_count, touch->velocity_x,
                            touch->velocity_y, state->touch_release_x,
                            state->touch_release_y, touch->first_x,
                            touch->first_y, touch->last_x, touch->last_y,
                            touch->touch_major, touch->touch_minor,
                            touch->max_touch_major,
                            touch->max_touch_minor);
            }
        }
    } else if (event->type == EV_KEY && event->value != 0) {
        if (state->inertia_active && config->verbose)
            fprintf(stderr, "click: cancelling inertia\n");
        reset_gesture(state);
    } else if (event->type == EV_SYN && event->code == SYN_REPORT) {
        int64_t now = monotonic_ns();

        finish_touch_report(state, now);
        finish_physical_report(state, config, now);
        finish_raw_scroll_report(state, config);
    } else if (event->type == EV_SYN && event->code == SYN_DROPPED) {
        reset_gesture(state);
        reset_touch_state(state);
    }
}

static int monitor_device(const char *path,
                          struct libevdev_uinput *output,
                          const struct config *config)
{
    struct daemon_state state = {0};
    struct libevdev *device = NULL;
    struct pollfd pollfd = {0};
    int fd;
    int error;

    fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
        return -errno;

    error = libevdev_new_from_fd(fd, &device);
    if (error != 0) {
        close(fd);
        return error;
    }

    if (config->verbose)
        fprintf(stderr, "monitoring %s (%s)\n", path,
                libevdev_get_name(device));

    pollfd.fd = fd;
    pollfd.events = POLLIN;

    while (running) {
        struct input_event event;
        int64_t now = monotonic_ns();
        int timeout = calculate_poll_timeout(&state, config, now);
        int poll_result = poll(&pollfd, 1, timeout);

        if (poll_result < 0) {
            if (errno == EINTR)
                continue;
            error = -errno;
            break;
        }

        if (poll_result > 0 &&
            (pollfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            error = -ENODEV;
            break;
        }

        if (poll_result > 0 && (pollfd.revents & POLLIN)) {
            while ((error = libevdev_next_event(
                        device, LIBEVDEV_READ_FLAG_NORMAL, &event)) == 0) {
                process_event(&state, config, &event);
            }
            if (error != -EAGAIN) {
                if (error == LIBEVDEV_READ_STATUS_SYNC) {
                    error = resync_device(&state, device, config);
                    if (error != 0)
                        break;
                    continue;
                }
                break;
            }
            reconcile_touch_state(&state, device, config);
        }

        now = monotonic_ns();
        error = emit_generated_scroll(output, &state);
        if (error != 0)
            break;
        maybe_start_inertia(&state, config, now);

        if (state.inertia_active &&
            now - state.last_tick_ns >= TICK_MS * NS_PER_MS) {
            error = emit_inertia_tick(output, &state, config, now);
            if (error != 0)
                break;

            if (should_stop_inertia(&state, config, now)) {
                if (config->verbose)
                    fprintf(stderr, "inertia stop\n");
                reset_gesture(&state);
            }
        }
    }

    libevdev_free(device);
    close(fd);
    return running ? error : 0;
}

static void print_usage(FILE *stream, const char *program)
{
    fprintf(stream,
            "Usage: %s [options]\n"
            "\n"
            "Adds a decaying scroll tail to an Apple Magic Mouse without\n"
            "interfering with its normal pointer or button events.\n"
            "\n"
            "Options:\n"
            "  --device-name NAME       evdev name (default: Apple Inc. Magic Mouse)\n"
            "  --idle-ms N              release detection delay (default: 42)\n"
            "  --decay-ms N             exponential decay time (default: 280)\n"
            "  --gain N                 release velocity multiplier (default: 0.72)\n"
            "  --min-velocity N         minimum high-res units/s (default: 240)\n"
            "  --max-velocity N         velocity safety cap (default: 9000)\n"
            "  --max-duration-ms N      maximum tail duration (default: 1100)\n"
            "  --touch-scale N          raw touch-to-scroll scale (default: 0.55)\n"
            "  --min-flick-distance N   raw displacement required (default: 90)\n"
            "  --verbose                log gesture starts and cancellations\n"
            "  --help                   show this help\n",
            program);
}

static double parse_double(const char *option, const char *value,
                           double minimum, double maximum)
{
    char *end = NULL;
    double parsed;

    errno = 0;
    parsed = strtod(value, &end);
    if (errno != 0 || end == value || *end != '\0' ||
        !isfinite(parsed) || parsed < minimum || parsed > maximum) {
        fprintf(stderr, "invalid %s value: %s\n", option, value);
        exit(EXIT_FAILURE);
    }

    return parsed;
}

static void parse_options(int argc, char **argv, struct config *config)
{
    enum {
        OPT_IDLE_MS = 1000,
        OPT_DECAY_MS,
        OPT_GAIN,
        OPT_MIN_VELOCITY,
        OPT_MAX_VELOCITY,
        OPT_MAX_DURATION_MS,
        OPT_TOUCH_SCALE,
        OPT_MIN_FLICK_DISTANCE,
    };
    static const struct option options[] = {
        {"device-name", required_argument, NULL, 'd'},
        {"idle-ms", required_argument, NULL, OPT_IDLE_MS},
        {"decay-ms", required_argument, NULL, OPT_DECAY_MS},
        {"gain", required_argument, NULL, OPT_GAIN},
        {"min-velocity", required_argument, NULL, OPT_MIN_VELOCITY},
        {"max-velocity", required_argument, NULL, OPT_MAX_VELOCITY},
        {"max-duration-ms", required_argument, NULL, OPT_MAX_DURATION_MS},
        {"touch-scale", required_argument, NULL, OPT_TOUCH_SCALE},
        {"min-flick-distance", required_argument, NULL,
         OPT_MIN_FLICK_DISTANCE},
        {"verbose", no_argument, NULL, 'v'},
        {"help", no_argument, NULL, 'h'},
        {NULL, 0, NULL, 0},
    };
    int option;

    while ((option = getopt_long(argc, argv, "d:vh", options, NULL)) != -1) {
        switch (option) {
        case 'd':
            config->device_name = optarg;
            break;
        case 'v':
            config->verbose = true;
            break;
        case 'h':
            print_usage(stdout, argv[0]);
            exit(EXIT_SUCCESS);
        case OPT_IDLE_MS:
            config->idle_ms =
                parse_double("--idle-ms", optarg, 10.0, 250.0);
            break;
        case OPT_DECAY_MS:
            config->decay_ms =
                parse_double("--decay-ms", optarg, 30.0, 2000.0);
            break;
        case OPT_GAIN:
            config->gain = parse_double("--gain", optarg, 0.0, 3.0);
            break;
        case OPT_MIN_VELOCITY:
            config->min_velocity =
                parse_double("--min-velocity", optarg, 1.0, 20000.0);
            break;
        case OPT_MAX_VELOCITY:
            config->max_velocity =
                parse_double("--max-velocity", optarg, 100.0, 100000.0);
            break;
        case OPT_MAX_DURATION_MS:
            config->max_duration_ms =
                parse_double("--max-duration-ms", optarg, 50.0, 5000.0);
            break;
        case OPT_TOUCH_SCALE:
            config->touch_scale =
                parse_double("--touch-scale", optarg, 0.01, 5.0);
            break;
        case OPT_MIN_FLICK_DISTANCE:
            config->min_flick_distance =
                parse_double("--min-flick-distance", optarg, 1.0, 1000.0);
            break;
        default:
            print_usage(stderr, argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    if (optind != argc) {
        fprintf(stderr, "unexpected argument: %s\n", argv[optind]);
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char **argv)
{
    struct config config = {
        .device_name = "Apple Inc. Magic Mouse",
        .idle_ms = 42.0,
        .decay_ms = 280.0,
        .gain = 0.72,
        .min_velocity = 240.0,
        .max_velocity = 9000.0,
        .max_duration_ms = 1100.0,
        .touch_scale = 0.55,
        .min_flick_distance = 90.0,
        .verbose = false,
    };
    struct libevdev_uinput *output = NULL;
    char device_path[256];
    int error;

    parse_options(argc, argv, &config);
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    error = create_output(&output);
    if (error != 0) {
        fprintf(stderr, "cannot create virtual input device: %s\n",
                strerror(-error));
        return EXIT_FAILURE;
    }

    if (config.verbose)
        fprintf(stderr, "virtual device: %s\n",
                libevdev_uinput_get_devnode(output));

    while (running) {
        error = find_device(config.device_name, device_path,
                            sizeof(device_path));
        if (error != 0) {
            if (error == -EACCES)
                fprintf(stderr,
                        "cannot read input devices; check udev permissions\n");
            else if (config.verbose)
                fprintf(stderr, "waiting for %s\n", config.device_name);
            sleep(2);
            continue;
        }

        error = monitor_device(device_path, output, &config);
        if (running) {
            fprintf(stderr, "%s disconnected or failed: %s; retrying\n",
                    config.device_name, strerror(-error));
            sleep(1);
        }
    }

    libevdev_uinput_destroy(output);
    return EXIT_SUCCESS;
}
