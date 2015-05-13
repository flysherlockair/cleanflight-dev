#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"
#include "version.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/color.h"
#include "common/typeconversion.h"

#include "drivers/system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"

#include "io/escservo.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/rc_controls.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/beeper.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "config_vars.h"

const configValue_t configVarDefs[] = {
    { "looptime",                   VAR_UINT16 | MASTER_VALUE,  &masterConfig.looptime, 0, 9000 },
    { "emf_avoidance",              VAR_UINT8  | MASTER_VALUE,  &masterConfig.emf_avoidance, 0, 1 },

    { "mid_rc",                     VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.midrc, 1200, 1700 },
    { "min_check",                  VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.mincheck, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "max_check",                  VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.maxcheck, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "rssi_channel",               VAR_INT8   | MASTER_VALUE,  &masterConfig.rxConfig.rssi_channel, 0, MAX_SUPPORTED_RC_CHANNEL_COUNT },
    { "rssi_scale",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.rssi_scale, RSSI_SCALE_MIN, RSSI_SCALE_MAX },
    { "rssi_ppm_invert",            VAR_INT8   | MASTER_VALUE,  &masterConfig.rxConfig.rssi_ppm_invert, 0, 1 },
    { "input_filtering_mode",       VAR_INT8   | MASTER_VALUE,  &masterConfig.inputFilteringMode, 0, 1 },

    { "min_throttle",               VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.minthrottle, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "max_throttle",               VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.maxthrottle, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "min_command",                VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.mincommand, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "servo_center_pulse",         VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.servoCenterPulse, PWM_RANGE_ZERO, PWM_RANGE_MAX },

    { "3d_deadband_low",            VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.deadband3d_low, PWM_RANGE_ZERO, PWM_RANGE_MAX }, // FIXME upper limit should match code in the mixer, 1500 currently
    { "3d_deadband_high",           VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.deadband3d_high, PWM_RANGE_ZERO, PWM_RANGE_MAX }, // FIXME lower limit should match code in the mixer, 1500 currently,
    { "3d_neutral",                 VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.neutral3d, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "3d_deadband_throttle",       VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.deadband3d_throttle, PWM_RANGE_ZERO, PWM_RANGE_MAX },

    { "motor_pwm_rate",             VAR_UINT16 | MASTER_VALUE,  &masterConfig.motor_pwm_rate, 50, 32000 },
    { "servo_pwm_rate",             VAR_UINT16 | MASTER_VALUE,  &masterConfig.servo_pwm_rate, 50, 498 },

    { "retarded_arm",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.retarded_arm, 0, 1 },
    { "disarm_kill_switch",         VAR_UINT8  | MASTER_VALUE,  &masterConfig.disarm_kill_switch, 0, 1 },
    { "auto_disarm_delay",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.auto_disarm_delay, 0, 60 },
    { "small_angle",                VAR_UINT8  | MASTER_VALUE,  &masterConfig.small_angle, 0, 180 },

    { "fixedwing_althold_dir",      VAR_INT8   | MASTER_VALUE,  &masterConfig.airplaneConfig.fixedwing_althold_dir, -1, 1 },

    { "reboot_character",           VAR_UINT8  | MASTER_VALUE,  &masterConfig.serialConfig.reboot_character, 48, 126 },

#ifdef GPS
    { "gps_provider",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.gpsConfig.provider, 0, GPS_PROVIDER_MAX },
    { "gps_sbas_mode",              VAR_UINT8  | MASTER_VALUE,  &masterConfig.gpsConfig.sbasMode, 0, SBAS_MODE_MAX },
    { "gps_auto_config",            VAR_UINT8  | MASTER_VALUE,  &masterConfig.gpsConfig.autoConfig, GPS_AUTOCONFIG_OFF, GPS_AUTOCONFIG_ON },
    { "gps_auto_baud",              VAR_UINT8  | MASTER_VALUE,  &masterConfig.gpsConfig.autoBaud, GPS_AUTOBAUD_OFF, GPS_AUTOBAUD_ON },

    { "gps_pos_p",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDPOS], 0, 200 },
    { "gps_pos_i",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDPOS], 0, 200 },
    { "gps_pos_d",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDPOS], 0, 200 },
    { "gps_posr_p",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDPOSR], 0, 200 },
    { "gps_posr_i",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDPOSR], 0, 200 },
    { "gps_posr_d",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDPOSR], 0, 200 },
    { "gps_nav_p",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDNAVR], 0, 200 },
    { "gps_nav_i",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDNAVR], 0, 200 },
    { "gps_nav_d",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDNAVR], 0, 200 },
    { "gps_wp_radius",              VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.gps_wp_radius, 0, 2000 },
    { "nav_controls_heading",       VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_controls_heading, 0, 1 },
    { "nav_speed_min",              VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_speed_min, 10, 2000 },
    { "nav_speed_max",              VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_speed_max, 10, 2000 },
    { "nav_slew_rate",              VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_slew_rate, 0, 100 },
#endif

    { "serialrx_provider",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.serialrx_provider, 0, SERIALRX_PROVIDER_MAX },
    { "spektrum_sat_bind",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.spektrum_sat_bind, SPEKTRUM_SAT_BIND_DISABLED, SPEKTRUM_SAT_BIND_MAX},

    { "telemetry_switch",           VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.telemetry_switch, 0, 1 },
    { "telemetry_inversion",        VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.telemetry_inversion, 0, 1 },
    { "frsky_default_lattitude",    VAR_FLOAT  | MASTER_VALUE,  &masterConfig.telemetryConfig.gpsNoFixLatitude, -90.0, 90.0 },
    { "frsky_default_longitude",    VAR_FLOAT  | MASTER_VALUE,  &masterConfig.telemetryConfig.gpsNoFixLongitude, -180.0, 180.0 },
    { "frsky_coordinates_format",   VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.frsky_coordinate_format, 0, FRSKY_FORMAT_NMEA },
    { "frsky_unit",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.frsky_unit, 0, FRSKY_UNIT_IMPERIALS },
    { "frsky_vfas_precision",       VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.frsky_vfas_precision, FRSKY_VFAS_PRECISION_LOW, FRSKY_VFAS_PRECISION_HIGH },
    { "hott_alarm_sound_interval",  VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.hottAlarmSoundInterval, 0, 120 },

    { "battery_capacity",           VAR_UINT16 | MASTER_VALUE,  &masterConfig.batteryConfig.batteryCapacity, 0, 20000 },
    { "vbat_scale",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatscale, VBAT_SCALE_MIN, VBAT_SCALE_MAX },
    { "vbat_max_cell_voltage",      VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatmaxcellvoltage, 10, 50 },
    { "vbat_min_cell_voltage",      VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatmincellvoltage, 10, 50 },
    { "vbat_warning_cell_voltage",  VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatwarningcellvoltage, 10, 50 },
    { "current_meter_scale",        VAR_INT16  | MASTER_VALUE,  &masterConfig.batteryConfig.currentMeterScale, -10000, 10000 },
    { "current_meter_offset",       VAR_UINT16 | MASTER_VALUE,  &masterConfig.batteryConfig.currentMeterOffset, 0, 3300 },
    { "multiwii_current_meter_output", VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.multiwiiCurrentMeterOutput, 0, 1 },
    { "current_meter_type",         VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.currentMeterType, 0, CURRENT_SENSOR_MAX },

    { "align_gyro",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.sensorAlignmentConfig.gyro_align, 0, 8 },
    { "align_acc",                  VAR_UINT8  | MASTER_VALUE,  &masterConfig.sensorAlignmentConfig.acc_align, 0, 8 },
    { "align_mag",                  VAR_UINT8  | MASTER_VALUE,  &masterConfig.sensorAlignmentConfig.mag_align, 0, 8 },

    { "align_board_roll",           VAR_INT16  | MASTER_VALUE,  &masterConfig.boardAlignment.rollDegrees, -180, 360 },
    { "align_board_pitch",          VAR_INT16  | MASTER_VALUE,  &masterConfig.boardAlignment.pitchDegrees, -180, 360 },
    { "align_board_yaw",            VAR_INT16  | MASTER_VALUE,  &masterConfig.boardAlignment.yawDegrees, -180, 360 },

    { "max_angle_inclination",      VAR_UINT16 | MASTER_VALUE,  &masterConfig.max_angle_inclination, 100, 900 },

    { "gyro_lpf",                   VAR_UINT16 | MASTER_VALUE,  &masterConfig.gyro_lpf, 0, 256 },
    { "moron_threshold",            VAR_UINT8  | MASTER_VALUE,  &masterConfig.gyroConfig.gyroMovementCalibrationThreshold, 0, 128 },
    { "gyro_cmpf_factor",           VAR_UINT16 | MASTER_VALUE,  &masterConfig.gyro_cmpf_factor, 100, 1000 },
    { "gyro_cmpfm_factor",          VAR_UINT16 | MASTER_VALUE,  &masterConfig.gyro_cmpfm_factor, 100, 1000 },

    { "alt_hold_deadband",          VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.alt_hold_deadband, 1, 250 },
    { "alt_hold_fast_change",       VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.alt_hold_fast_change, 0, 1 },
    { "deadband",                   VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.deadband, 0, 32 },
    { "yaw_deadband",               VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.yaw_deadband, 0, 100 },

    { "throttle_correction_value",  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].throttle_correction_value, 0, 150 },
    { "throttle_correction_angle",  VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].throttle_correction_angle, 1, 900 },

    { "yaw_control_direction",      VAR_INT8   | MASTER_VALUE,  &masterConfig.yaw_control_direction, -1, 1 },

    { "pid_at_min_throttle",        VAR_UINT8  | MASTER_VALUE, &masterConfig.mixerConfig.pid_at_min_throttle, 0, 1 },
    { "yaw_motor_direction",        VAR_INT8   | MASTER_VALUE, &masterConfig.mixerConfig.yaw_motor_direction, -1, 1 },
    { "yaw_jump_prevention_limit",  VAR_UINT16 | MASTER_VALUE, &masterConfig.mixerConfig.yaw_jump_prevention_limit, YAW_JUMP_PREVENTION_LIMIT_LOW, YAW_JUMP_PREVENTION_LIMIT_HIGH },
#ifdef USE_SERVOS
    { "tri_unarmed_servo",          VAR_INT8   | MASTER_VALUE, &masterConfig.mixerConfig.tri_unarmed_servo, 0, 1 },
    { "servo_lowpass_freq",         VAR_INT16  | MASTER_VALUE, &masterConfig.mixerConfig.servo_lowpass_freq, 10, 400},
    { "servo_lowpass_enable",       VAR_INT8   | MASTER_VALUE, &masterConfig.mixerConfig.servo_lowpass_enable, 0, 1 },
#endif

    { "default_rate_profile",       VAR_UINT8  | PROFILE_VALUE , &masterConfig.profile[0].defaultRateProfileIndex, 0, MAX_CONTROL_RATE_PROFILE_COUNT - 1 },
    { "rc_rate",                    VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcRate8, 0, 250 },
    { "rc_expo",                    VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcExpo8, 0, 100 },
    { "rc_yaw_expo",                VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcYawExpo8, 0, 100 },
    { "thr_mid",                    VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].thrMid8, 0, 100 },
    { "thr_expo",                   VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].thrExpo8, 0, 100 },
    { "roll_rate",                  VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_ROLL], 0, CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX },
    { "pitch_rate",                 VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_PITCH], 0, CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX },
    { "yaw_rate",                   VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_YAW], 0, CONTROL_RATE_CONFIG_YAW_RATE_MAX },
    { "tpa_rate",                   VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].dynThrPID, 0, CONTROL_RATE_CONFIG_TPA_MAX},
    { "tpa_breakpoint",             VAR_UINT16 | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].tpa_breakpoint, PWM_RANGE_MIN, PWM_RANGE_MAX},

    { "failsafe_delay",             VAR_UINT8  | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_delay, 0, 200 },
    { "failsafe_off_delay",         VAR_UINT8  | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_off_delay, 0, 200 },
    { "failsafe_throttle",          VAR_UINT16 | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_throttle, PWM_RANGE_MIN, PWM_RANGE_MAX },
    { "failsafe_kill_switch",       VAR_UINT8  | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_kill_switch, 0, 1 },
    { "failsafe_throttle_low_delay",VAR_UINT16 | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_throttle_low_delay, 0, 300 },

    { "rx_min_usec",                VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.rx_min_usec, PWM_PULSE_MIN, PWM_PULSE_MAX },
    { "rx_max_usec",                VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.rx_max_usec, PWM_PULSE_MIN, PWM_PULSE_MAX },

#ifdef USE_SERVOS
    { "gimbal_mode",                VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].gimbalConfig.mode, 0, GIMBAL_MODE_MAX},
#endif

    { "acc_hardware",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.acc_hardware, 0, ACC_MAX },
    { "acc_lpf_factor",             VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].acc_lpf_factor, 0, 250 },
    { "accxy_deadband",             VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].accDeadband.xy, 0, 100 },
    { "accz_deadband",              VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].accDeadband.z, 0, 100 },
    { "accz_lpf_cutoff",            VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].accz_lpf_cutoff, 1, 20 },
    { "acc_unarmedcal",             VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].acc_unarmedcal, 0, 1 },
    { "acc_trim_pitch",             VAR_INT16  | PROFILE_VALUE, &masterConfig.profile[0].accelerometerTrims.values.pitch, -300, 300 },
    { "acc_trim_roll",              VAR_INT16  | PROFILE_VALUE, &masterConfig.profile[0].accelerometerTrims.values.roll, -300, 300 },

    { "baro_tab_size",              VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_sample_count, 0, BARO_SAMPLE_COUNT_MAX },
    { "baro_noise_lpf",             VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_noise_lpf, 0, 1 },
    { "baro_cf_vel",                VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_cf_vel, 0, 1 },
    { "baro_cf_alt",                VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_cf_alt, 0, 1 },
    { "baro_hardware",              VAR_UINT8  | MASTER_VALUE,  &masterConfig.baro_hardware, 0, BARO_MAX },

    { "mag_hardware",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.mag_hardware, 0, MAG_MAX },
    { "mag_declination",            VAR_INT16  | PROFILE_VALUE, &masterConfig.profile[0].mag_declination, -18000, 18000 },

    { "pid_controller",             VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.pidController, 0, 5 },

    { "p_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PITCH], 0, 200 },
    { "i_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PITCH], 0, 200 },
    { "d_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PITCH], 0, 200 },
    { "p_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[ROLL], 0, 200 },
    { "i_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[ROLL], 0, 200 },
    { "d_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[ROLL], 0, 200 },
    { "p_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[YAW], 0, 200 },
    { "i_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[YAW], 0, 200 },
    { "d_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[YAW], 0, 200 },

    { "p_pitchf",                   VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[PITCH], 0, 100 },
    { "i_pitchf",                   VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[PITCH], 0, 100 },
    { "d_pitchf",                   VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[PITCH], 0, 100 },
    { "p_rollf",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[ROLL], 0, 100 },
    { "i_rollf",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[ROLL], 0, 100 },
    { "d_rollf",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[ROLL], 0, 100 },
    { "p_yawf",                     VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[YAW], 0, 100 },
    { "i_yawf",                     VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[YAW], 0, 100 },
    { "d_yawf",                     VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[YAW], 0, 100 },

    { "level_horizon",              VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.H_level, 0, 10 },
    { "level_angle",                VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.A_level, 0, 10 },
    { "sensitivity_horizon",        VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.H_sensitivity, 0, 250 },

    { "p_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDALT], 0, 200 },
    { "i_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDALT], 0, 200 },
    { "d_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDALT], 0, 200 },

    { "p_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDLEVEL], 0, 200 },
    { "i_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDLEVEL], 0, 200 },
    { "d_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDLEVEL], 0, 200 },

    { "p_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDVEL], 0, 200 },
    { "i_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDVEL], 0, 200 },
    { "d_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDVEL], 0, 200 },

    { "yaw_p_limit",                VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.yaw_p_limit, YAW_P_LIMIT_MIN, YAW_P_LIMIT_MAX },
    { "dterm_cut_hz",               VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.dterm_cut_hz, 0, 200 },
    { "pterm_cut_hz",               VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.pterm_cut_hz, 0, 200 },
    { "gyro_cut_hz",                VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.gyro_cut_hz, 0, 200 },

#ifdef BLACKBOX
    { "blackbox_rate_num",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.blackbox_rate_num, 1, 32 },
    { "blackbox_rate_denom",        VAR_UINT8  | MASTER_VALUE,  &masterConfig.blackbox_rate_denom, 1, 32 },
    { "blackbox_device",            VAR_UINT8  | MASTER_VALUE,  &masterConfig.blackbox_device, 0, 1 },
#endif

    { "magzero_x",                  VAR_INT16  | MASTER_VALUE, &masterConfig.magZero.raw[X], -32768, 32767 },
    { "magzero_y",                  VAR_INT16  | MASTER_VALUE, &masterConfig.magZero.raw[Y], -32768, 32767 },
    { "magzero_z",                  VAR_INT16  | MASTER_VALUE, &masterConfig.magZero.raw[Z], -32768, 32767 },
};

const uint16_t configVarDefCount = sizeof(configVarDefs) / sizeof(configVarDefs[0]);

/**
 * Get a pointer to the current value of the configuration variable given the currently selected profile/rate profile.
 */
void* configVarResolveValuePointer(const configValue_t *value)
{
    void *ptr = value->ptr;

    if (value->type & PROFILE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index);
    }

    if (value->type & CONTROL_RATE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(controlRateConfig_t) * getCurrentControlRateProfile());
    }

    return ptr;
}
