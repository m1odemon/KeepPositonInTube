#ifndef BALL_CONTROL_H
#define BALL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Pure one-dimensional ball controller.
 *
 * The Raspberry Pi supplies timestamped ball positions.  This module estimates
 * ball velocity and runs a position-PI / velocity-PD cascade whose output is a
 * requested tube angle.  UART, PWM, linkage calibration, interrupts, and the
 * main loop intentionally remain outside this module.
 */

typedef enum
{
    BALL_CONTROL_IDLE = 0,
    BALL_CONTROL_ARMING,
    BALL_CONTROL_TRACKING,
    BALL_CONTROL_HOLD,
    BALL_CONTROL_FAULT_LATCHED
} ball_control_state_enum;

typedef enum
{
    BALL_CONTROL_FAULT_NONE = 0,
    BALL_CONTROL_FAULT_VISION_TIMEOUT,
    BALL_CONTROL_FAULT_EMERGENCY_STOP,
    BALL_CONTROL_FAULT_POSITION_HARD_LIMIT
} ball_control_fault_enum;

typedef struct
{
    uint32_t sequence;
    uint32_t capture_timestamp_us;
    float position_mm;
    float confidence;
} ball_control_measurement_t;

typedef struct
{
    float position_min_mm;
    float position_max_mm;
    float position_filter_tau_s;
    float velocity_filter_tau_s;
    float acceleration_filter_tau_s;
    float min_confidence;
    float max_valid_speed_mm_s;
    float max_position_innovation_mm;

    /* Position PI: position error (mm) -> target ball speed (mm/s). */
    float position_kp_per_s;
    float position_ki_per_s2;
    float position_integral_zone_mm;
    float position_integral_limit_mm_s;
    float target_speed_limit_mm_s;
    float hold_target_speed_limit_mm_s;

    /*
     * Velocity PD: speed error (mm/s) -> requested tube angle (degree).
     * The D term acts on measured ball acceleration to avoid setpoint kick.
     */
    float velocity_kp_deg_s_per_mm;
    float velocity_kd_deg_s2_per_mm;
    float tube_angle_limit_deg;
    float hold_tube_angle_limit_deg;
    float min_effective_angle_pos_deg;
    float min_effective_angle_neg_deg;
    float tube_angle_rate_limit_deg_s;

    /*
     * Optional car-acceleration feedforward.  Keep zero until the longitudinal
     * acceleration sign and scale have been verified on the real vehicle.
     */
    float vehicle_accel_feedforward_deg_s2_per_mm;

    float position_tolerance_mm;
    float exit_hold_position_error_mm;
    float velocity_tolerance_mm_s;
    float dt_min_s;
    float dt_max_s;
    uint16_t vision_timeout_ms;
    uint16_t settle_time_ms;
    uint8_t arming_valid_frames;
} ball_control_config_t;

typedef struct
{
    ball_control_config_t config;
    ball_control_state_enum state;
    ball_control_fault_enum fault;
    float target_position_mm;
    float filtered_position_mm;
    float filtered_velocity_mm_s;
    float filtered_acceleration_mm_s2;
    float last_raw_position_mm;
    float position_integral_mm_s;
    float target_velocity_mm_s;
    float target_tube_angle_deg;
    float vehicle_acceleration_mm_s2;
    uint32_t last_sequence;
    uint32_t last_capture_timestamp_us;
    uint32_t last_valid_receive_timestamp_us;
    uint32_t arming_started_timestamp_us;
    uint32_t last_output_timestamp_us;
    uint32_t settle_elapsed_us;
    uint8_t arming_frame_count;
    bool enable_requested;
    bool estimator_initialized;
    bool velocity_valid;
    bool acceleration_valid;
    bool sequence_valid;
    bool receive_timestamp_valid;
    bool arming_started_timestamp_valid;
    bool output_timestamp_valid;
} ball_control_t;

void ball_control_config_defaults(ball_control_config_t *config);
void ball_control_init(ball_control_t *controller, const ball_control_config_t *config);

void ball_control_set_target_mm(ball_control_t *controller, float target_position_mm);
void ball_control_set_vehicle_acceleration_mm_s2(ball_control_t *controller,
                                                  float acceleration_mm_s2);
void ball_control_request_enable(ball_control_t *controller, bool enable, uint32_t now_us);
bool ball_control_reset_fault(ball_control_t *controller);
void ball_control_emergency_stop(ball_control_t *controller, uint32_t now_us);

/*
 * Consume exactly one complete measurement in the controller context.
 * receive_timestamp_us must come from the MCU's local monotonic timer.
 * Returns true only when the measurement updated the estimator.
 */
bool ball_control_consume_measurement(ball_control_t *controller,
                                      const ball_control_measurement_t *measurement,
                                      uint32_t receive_timestamp_us);

/* Call periodically (for example, every 5 ms) for timeout/fault handling. */
void ball_control_tick(ball_control_t *controller, uint32_t now_us);

float ball_control_get_target_velocity_mm_s(const ball_control_t *controller);
float ball_control_get_target_tube_angle_deg(const ball_control_t *controller);

#endif
