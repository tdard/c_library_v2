#pragma once
// MESSAGE GDP_UAV_STATE PACKING

#define MAVLINK_MSG_ID_GDP_UAV_STATE 50100

MAVPACKED(
typedef struct __mavlink_gdp_uav_state_t {
 uint64_t timestamp; /*<  Timestamp*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t frame; /*<  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.*/
 uint8_t state; /*<  UAV State*/
}) mavlink_gdp_uav_state_t;

#define MAVLINK_MSG_ID_GDP_UAV_STATE_LEN 12
#define MAVLINK_MSG_ID_GDP_UAV_STATE_MIN_LEN 12
#define MAVLINK_MSG_ID_50100_LEN 12
#define MAVLINK_MSG_ID_50100_MIN_LEN 12

#define MAVLINK_MSG_ID_GDP_UAV_STATE_CRC 72
#define MAVLINK_MSG_ID_50100_CRC 72



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GDP_UAV_STATE { \
    50100, \
    "GDP_UAV_STATE", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gdp_uav_state_t, timestamp) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gdp_uav_state_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gdp_uav_state_t, target_component) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gdp_uav_state_t, frame) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gdp_uav_state_t, state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GDP_UAV_STATE { \
    "GDP_UAV_STATE", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gdp_uav_state_t, timestamp) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gdp_uav_state_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gdp_uav_state_t, target_component) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gdp_uav_state_t, frame) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gdp_uav_state_t, state) }, \
         } \
}
#endif

/**
 * @brief Pack a gdp_uav_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Timestamp
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param state  UAV State
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gdp_uav_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t target_system, uint8_t target_component, uint8_t frame, uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GDP_UAV_STATE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, frame);
    _mav_put_uint8_t(buf, 11, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN);
#else
    mavlink_gdp_uav_state_t packet;
    packet.timestamp = timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GDP_UAV_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GDP_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_CRC);
}

/**
 * @brief Pack a gdp_uav_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Timestamp
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param state  UAV State
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gdp_uav_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t target_system,uint8_t target_component,uint8_t frame,uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GDP_UAV_STATE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, frame);
    _mav_put_uint8_t(buf, 11, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN);
#else
    mavlink_gdp_uav_state_t packet;
    packet.timestamp = timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GDP_UAV_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GDP_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_CRC);
}

/**
 * @brief Encode a gdp_uav_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gdp_uav_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gdp_uav_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gdp_uav_state_t* gdp_uav_state)
{
    return mavlink_msg_gdp_uav_state_pack(system_id, component_id, msg, gdp_uav_state->timestamp, gdp_uav_state->target_system, gdp_uav_state->target_component, gdp_uav_state->frame, gdp_uav_state->state);
}

/**
 * @brief Encode a gdp_uav_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gdp_uav_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gdp_uav_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gdp_uav_state_t* gdp_uav_state)
{
    return mavlink_msg_gdp_uav_state_pack_chan(system_id, component_id, chan, msg, gdp_uav_state->timestamp, gdp_uav_state->target_system, gdp_uav_state->target_component, gdp_uav_state->frame, gdp_uav_state->state);
}

/**
 * @brief Send a gdp_uav_state message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Timestamp
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param state  UAV State
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gdp_uav_state_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t target_system, uint8_t target_component, uint8_t frame, uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GDP_UAV_STATE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, frame);
    _mav_put_uint8_t(buf, 11, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GDP_UAV_STATE, buf, MAVLINK_MSG_ID_GDP_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_CRC);
#else
    mavlink_gdp_uav_state_t packet;
    packet.timestamp = timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;
    packet.state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GDP_UAV_STATE, (const char *)&packet, MAVLINK_MSG_ID_GDP_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_CRC);
#endif
}

/**
 * @brief Send a gdp_uav_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gdp_uav_state_send_struct(mavlink_channel_t chan, const mavlink_gdp_uav_state_t* gdp_uav_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gdp_uav_state_send(chan, gdp_uav_state->timestamp, gdp_uav_state->target_system, gdp_uav_state->target_component, gdp_uav_state->frame, gdp_uav_state->state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GDP_UAV_STATE, (const char *)gdp_uav_state, MAVLINK_MSG_ID_GDP_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_GDP_UAV_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gdp_uav_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t target_system, uint8_t target_component, uint8_t frame, uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, frame);
    _mav_put_uint8_t(buf, 11, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GDP_UAV_STATE, buf, MAVLINK_MSG_ID_GDP_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_CRC);
#else
    mavlink_gdp_uav_state_t *packet = (mavlink_gdp_uav_state_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->frame = frame;
    packet->state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GDP_UAV_STATE, (const char *)packet, MAVLINK_MSG_ID_GDP_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN, MAVLINK_MSG_ID_GDP_UAV_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE GDP_UAV_STATE UNPACKING


/**
 * @brief Get field timestamp from gdp_uav_state message
 *
 * @return  Timestamp
 */
static inline uint64_t mavlink_msg_gdp_uav_state_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_system from gdp_uav_state message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_gdp_uav_state_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field target_component from gdp_uav_state message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_gdp_uav_state_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field frame from gdp_uav_state message
 *
 * @return  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 */
static inline uint8_t mavlink_msg_gdp_uav_state_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field state from gdp_uav_state message
 *
 * @return  UAV State
 */
static inline uint8_t mavlink_msg_gdp_uav_state_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Decode a gdp_uav_state message into a struct
 *
 * @param msg The message to decode
 * @param gdp_uav_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_gdp_uav_state_decode(const mavlink_message_t* msg, mavlink_gdp_uav_state_t* gdp_uav_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gdp_uav_state->timestamp = mavlink_msg_gdp_uav_state_get_timestamp(msg);
    gdp_uav_state->target_system = mavlink_msg_gdp_uav_state_get_target_system(msg);
    gdp_uav_state->target_component = mavlink_msg_gdp_uav_state_get_target_component(msg);
    gdp_uav_state->frame = mavlink_msg_gdp_uav_state_get_frame(msg);
    gdp_uav_state->state = mavlink_msg_gdp_uav_state_get_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GDP_UAV_STATE_LEN? msg->len : MAVLINK_MSG_ID_GDP_UAV_STATE_LEN;
        memset(gdp_uav_state, 0, MAVLINK_MSG_ID_GDP_UAV_STATE_LEN);
    memcpy(gdp_uav_state, _MAV_PAYLOAD(msg), len);
#endif
}
