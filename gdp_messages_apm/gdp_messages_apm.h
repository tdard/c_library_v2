/** @file
 *  @brief MAVLink comm protocol generated from gdp_messages_apm.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_GDP_MESSAGES_APM_H
#define MAVLINK_GDP_MESSAGES_APM_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_GDP_MESSAGES_APM.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{150, 134, 42, 0, 0, 0}, {151, 219, 8, 3, 6, 7}, {152, 208, 4, 0, 0, 0}, {153, 188, 12, 0, 0, 0}, {154, 84, 15, 3, 6, 7}, {155, 22, 13, 3, 4, 5}, {156, 19, 6, 3, 0, 1}, {157, 21, 15, 3, 12, 13}, {158, 134, 14, 3, 12, 13}, {160, 78, 12, 3, 8, 9}, {161, 68, 3, 3, 0, 1}, {162, 189, 8, 0, 0, 0}, {163, 127, 28, 0, 0, 0}, {164, 154, 44, 0, 0, 0}, {165, 21, 3, 0, 0, 0}, {166, 21, 9, 0, 0, 0}, {167, 144, 22, 0, 0, 0}, {168, 1, 12, 0, 0, 0}, {169, 234, 18, 0, 0, 0}, {170, 73, 34, 0, 0, 0}, {171, 181, 66, 0, 0, 0}, {172, 22, 98, 0, 0, 0}, {173, 83, 8, 0, 0, 0}, {174, 167, 48, 0, 0, 0}, {175, 138, 19, 3, 14, 15}, {176, 234, 3, 3, 0, 1}, {177, 240, 20, 0, 0, 0}, {178, 47, 24, 0, 0, 0}, {179, 189, 29, 1, 26, 0}, {180, 52, 45, 1, 42, 0}, {181, 174, 4, 0, 0, 0}, {182, 229, 40, 0, 0, 0}, {183, 85, 2, 3, 0, 1}, {184, 159, 206, 3, 4, 5}, {185, 186, 7, 3, 4, 5}, {186, 72, 29, 3, 0, 1}, {191, 92, 27, 0, 0, 0}, {192, 36, 44, 0, 0, 0}, {193, 71, 22, 0, 0, 0}, {194, 98, 25, 0, 0, 0}, {195, 120, 37, 0, 0, 0}, {200, 134, 42, 3, 40, 41}, {201, 205, 14, 3, 12, 13}, {214, 69, 8, 3, 6, 7}, {215, 101, 3, 0, 0, 0}, {216, 50, 3, 3, 0, 1}, {217, 202, 6, 0, 0, 0}, {218, 17, 7, 3, 0, 1}, {219, 162, 2, 0, 0, 0}, {226, 207, 8, 0, 0, 0}, {11000, 134, 51, 3, 4, 5}, {11001, 15, 135, 0, 0, 0}, {11002, 234, 179, 3, 4, 5}, {11003, 64, 5, 0, 0, 0}, {11010, 46, 49, 0, 0, 0}, {11011, 106, 44, 0, 0, 0}, {11020, 205, 16, 0, 0, 0}, {11030, 144, 44, 0, 0, 0}, {11031, 133, 44, 0, 0, 0}, {11032, 85, 44, 0, 0, 0}, {50100, 72, 12, 3, 8, 9}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_GDP_MESSAGES_APM

// ENUM DEFINITIONS


/** @brief Description of the state of the UAV used in the Group Design Project of AVDC Student of Cranfield university */
#ifndef HAVE_ENUM_GDP_UAV_STATE_ENUM
#define HAVE_ENUM_GDP_UAV_STATE_ENUM
typedef enum GDP_UAV_STATE_ENUM
{
   GDP_UAV_STATE_TRACK=0, /* UAV is tracking an enemy target | */
   GDP_UAV_STATE_AVOID=1, /* UAV is avoiding any drone it detects | */
   GDP_UAV_STATE_DEFEND=2, /* UAV is patrolling in a defense zone | */
   GDP_UAV_STATE_ATTACK_PATROL=3, /* UAV is patrolling in an attack zone | */
   GDP_UAV_STATE_ATTACK_DROP=4, /* UAV is going to drone a payload in a zone to attack | */
   GDP_UAV_STATE_ENUM_ENUM_END=5, /*  | */
} GDP_UAV_STATE_ENUM;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_CMD
#define HAVE_ENUM_MAV_CMD
typedef enum MAV_CMD
{
   MAV_CMD_NAV_ALTITUDE_WAIT=83, /* Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. |Altitude (m).| Descent speed (m/s).| Wiggle Time (s).| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_DO_GRIPPER=211, /* Mission command to operate EPM gripper. |Gripper number (a number from 1 to max number of grippers on the vehicle).| Gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum).| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_DO_AUTOTUNE_ENABLE=212, /* Enable/disable autotune. |Enable (1: enable, 0:disable).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_POWER_OFF_INITIATED=42000, /* A system wide power-off event has been initiated. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_SOLO_BTN_FLY_CLICK=42001, /* FLY button has been clicked. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_SOLO_BTN_FLY_HOLD=42002, /* FLY button has been held for 1.5 seconds. |Takeoff altitude.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_SOLO_BTN_PAUSE_CLICK=42003, /* PAUSE button has been clicked. |1 if Solo is in a shot mode, 0 otherwise.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_FIXED_MAG_CAL=42004, /* Magnetometer calibration based on fixed position
        in earth field given by inclination, declination and intensity. |MagDeclinationDegrees.| MagInclinationDegrees.| MagIntensityMilliGauss.| YawDegrees.| Empty.| Empty.| Empty.|  */
   MAV_CMD_FIXED_MAG_CAL_FIELD=42005, /* Magnetometer calibration based on fixed expected field values in milliGauss. |FieldX.| FieldY.| FieldZ.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_DO_START_MAG_CAL=42424, /* Initiate a magnetometer calibration. |uint8_t bitmask of magnetometers (0 means all).| Automatically retry on failure (0=no retry, 1=retry).| Save without user input (0=require input, 1=autosave).| Delay (seconds).| Autoreboot (0=user reboot, 1=autoreboot).| Empty.| Empty.|  */
   MAV_CMD_DO_ACCEPT_MAG_CAL=42425, /* Initiate a magnetometer calibration. |uint8_t bitmask of magnetometers (0 means all).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_DO_CANCEL_MAG_CAL=42426, /* Cancel a running magnetometer calibration. |uint8_t bitmask of magnetometers (0 means all).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_SET_FACTORY_TEST_MODE=42427, /* Command autopilot to get into factory test/diagnostic mode. |0 means get out of test mode, 1 means get into test mode.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_DO_SEND_BANNER=42428, /* Reply with the version banner. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_ACCELCAL_VEHICLE_POS=42429, /* Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in. |Position, one of the ACCELCAL_VEHICLE_POS enum values.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_GIMBAL_RESET=42501, /* Causes the gimbal to reset and boot as if it was just powered on. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS=42502, /* Reports progress and success or failure of gimbal axis calibration procedure. |Gimbal axis we're reporting calibration progress for.| Current calibration progress for this axis, 0x64=100%.| Status of the calibration.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION=42503, /* Starts commutation calibration on the gimbal. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
   MAV_CMD_GIMBAL_FULL_RESET=42505, /* Erases gimbal application and parameters. |Magic number.| Magic number.| Magic number.| Magic number.| Magic number.| Magic number.| Magic number.|  */
   MAV_CMD_DO_WINCH=42600, /* Command to operate winch. |Winch number (0 for the default winch, otherwise a number from 1 to max number of winches on the vehicle).| Action (0=relax, 1=relative length control, 2=rate control. See WINCH_ACTIONS enum.).| Release length (cable distance to unwind in meters, negative numbers to wind in cable).| Release rate (meters/second).| Empty.| Empty.| Empty.|  */
   MAV_CMD_FLASH_BOOTLOADER=42650, /* Update the bootloader |Empty| Empty| Empty| Empty| Magic number - set to 290876 to actually flash| Empty| Empty|  */
   MAV_CMD_NAV_GDP_PAYLOAD_PLACE=50000, /* Descend and place payload. Vehicle descends until it detects a hanging payload has reached the ground, the gripper is opened to release the payload |Empty| Empty| Empty| Empty| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  */
   MAV_CMD_REQUEST_GDP_UAV_STATE=50010, /* Request the UAV State described by GDP_UAV_STATE |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_ENUM_END=50011, /*  | */
} MAV_CMD;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_gdp_uav_state.h"

// base include
#include "../ardupilotmega/ardupilotmega.h"

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_SENSOR_OFFSETS, MAVLINK_MESSAGE_INFO_SET_MAG_OFFSETS, MAVLINK_MESSAGE_INFO_MEMINFO, MAVLINK_MESSAGE_INFO_AP_ADC, MAVLINK_MESSAGE_INFO_DIGICAM_CONFIGURE, MAVLINK_MESSAGE_INFO_DIGICAM_CONTROL, MAVLINK_MESSAGE_INFO_MOUNT_CONFIGURE, MAVLINK_MESSAGE_INFO_MOUNT_CONTROL, MAVLINK_MESSAGE_INFO_MOUNT_STATUS, MAVLINK_MESSAGE_INFO_FENCE_POINT, MAVLINK_MESSAGE_INFO_FENCE_FETCH_POINT, MAVLINK_MESSAGE_INFO_FENCE_STATUS, MAVLINK_MESSAGE_INFO_AHRS, MAVLINK_MESSAGE_INFO_SIMSTATE, MAVLINK_MESSAGE_INFO_HWSTATUS, MAVLINK_MESSAGE_INFO_RADIO, MAVLINK_MESSAGE_INFO_LIMITS_STATUS, MAVLINK_MESSAGE_INFO_WIND, MAVLINK_MESSAGE_INFO_DATA16, MAVLINK_MESSAGE_INFO_DATA32, MAVLINK_MESSAGE_INFO_DATA64, MAVLINK_MESSAGE_INFO_DATA96, MAVLINK_MESSAGE_INFO_RANGEFINDER, MAVLINK_MESSAGE_INFO_AIRSPEED_AUTOCAL, MAVLINK_MESSAGE_INFO_RALLY_POINT, MAVLINK_MESSAGE_INFO_RALLY_FETCH_POINT, MAVLINK_MESSAGE_INFO_COMPASSMOT_STATUS, MAVLINK_MESSAGE_INFO_AHRS2, MAVLINK_MESSAGE_INFO_CAMERA_STATUS, MAVLINK_MESSAGE_INFO_CAMERA_FEEDBACK, MAVLINK_MESSAGE_INFO_BATTERY2, MAVLINK_MESSAGE_INFO_AHRS3, MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION_REQUEST, MAVLINK_MESSAGE_INFO_REMOTE_LOG_DATA_BLOCK, MAVLINK_MESSAGE_INFO_REMOTE_LOG_BLOCK_STATUS, MAVLINK_MESSAGE_INFO_LED_CONTROL, MAVLINK_MESSAGE_INFO_MAG_CAL_PROGRESS, MAVLINK_MESSAGE_INFO_MAG_CAL_REPORT, MAVLINK_MESSAGE_INFO_EKF_STATUS_REPORT, MAVLINK_MESSAGE_INFO_PID_TUNING, MAVLINK_MESSAGE_INFO_DEEPSTALL, MAVLINK_MESSAGE_INFO_GIMBAL_REPORT, MAVLINK_MESSAGE_INFO_GIMBAL_CONTROL, MAVLINK_MESSAGE_INFO_GIMBAL_TORQUE_CMD_REPORT, MAVLINK_MESSAGE_INFO_GOPRO_HEARTBEAT, MAVLINK_MESSAGE_INFO_GOPRO_GET_REQUEST, MAVLINK_MESSAGE_INFO_GOPRO_GET_RESPONSE, MAVLINK_MESSAGE_INFO_GOPRO_SET_REQUEST, MAVLINK_MESSAGE_INFO_GOPRO_SET_RESPONSE, MAVLINK_MESSAGE_INFO_RPM, MAVLINK_MESSAGE_INFO_DEVICE_OP_READ, MAVLINK_MESSAGE_INFO_DEVICE_OP_READ_REPLY, MAVLINK_MESSAGE_INFO_DEVICE_OP_WRITE, MAVLINK_MESSAGE_INFO_DEVICE_OP_WRITE_REPLY, MAVLINK_MESSAGE_INFO_ADAP_TUNING, MAVLINK_MESSAGE_INFO_VISION_POSITION_DELTA, MAVLINK_MESSAGE_INFO_AOA_SSA, MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_1_TO_4, MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_5_TO_8, MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_9_TO_12, MAVLINK_MESSAGE_INFO_GDP_UAV_STATE}
# define MAVLINK_MESSAGE_NAMES {{ "ADAP_TUNING", 11010 }, { "AHRS", 163 }, { "AHRS2", 178 }, { "AHRS3", 182 }, { "AIRSPEED_AUTOCAL", 174 }, { "AOA_SSA", 11020 }, { "AP_ADC", 153 }, { "AUTOPILOT_VERSION_REQUEST", 183 }, { "BATTERY2", 181 }, { "CAMERA_FEEDBACK", 180 }, { "CAMERA_STATUS", 179 }, { "COMPASSMOT_STATUS", 177 }, { "DATA16", 169 }, { "DATA32", 170 }, { "DATA64", 171 }, { "DATA96", 172 }, { "DEEPSTALL", 195 }, { "DEVICE_OP_READ", 11000 }, { "DEVICE_OP_READ_REPLY", 11001 }, { "DEVICE_OP_WRITE", 11002 }, { "DEVICE_OP_WRITE_REPLY", 11003 }, { "DIGICAM_CONFIGURE", 154 }, { "DIGICAM_CONTROL", 155 }, { "EKF_STATUS_REPORT", 193 }, { "ESC_TELEMETRY_1_TO_4", 11030 }, { "ESC_TELEMETRY_5_TO_8", 11031 }, { "ESC_TELEMETRY_9_TO_12", 11032 }, { "FENCE_FETCH_POINT", 161 }, { "FENCE_POINT", 160 }, { "FENCE_STATUS", 162 }, { "GDP_UAV_STATE", 50100 }, { "GIMBAL_CONTROL", 201 }, { "GIMBAL_REPORT", 200 }, { "GIMBAL_TORQUE_CMD_REPORT", 214 }, { "GOPRO_GET_REQUEST", 216 }, { "GOPRO_GET_RESPONSE", 217 }, { "GOPRO_HEARTBEAT", 215 }, { "GOPRO_SET_REQUEST", 218 }, { "GOPRO_SET_RESPONSE", 219 }, { "HWSTATUS", 165 }, { "LED_CONTROL", 186 }, { "LIMITS_STATUS", 167 }, { "MAG_CAL_PROGRESS", 191 }, { "MAG_CAL_REPORT", 192 }, { "MEMINFO", 152 }, { "MOUNT_CONFIGURE", 156 }, { "MOUNT_CONTROL", 157 }, { "MOUNT_STATUS", 158 }, { "PID_TUNING", 194 }, { "RADIO", 166 }, { "RALLY_FETCH_POINT", 176 }, { "RALLY_POINT", 175 }, { "RANGEFINDER", 173 }, { "REMOTE_LOG_BLOCK_STATUS", 185 }, { "REMOTE_LOG_DATA_BLOCK", 184 }, { "RPM", 226 }, { "SENSOR_OFFSETS", 150 }, { "SET_MAG_OFFSETS", 151 }, { "SIMSTATE", 164 }, { "VISION_POSITION_DELTA", 11011 }, { "WIND", 168 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_GDP_MESSAGES_APM_H
