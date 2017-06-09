#pragma once
// MESSAGE RADAR_FILTERED PACKING

#define MAVLINK_MSG_ID_RADAR_FILTERED 181

MAVPACKED(
typedef struct __mavlink_radar_filtered_t {
 float distance[5]; /*< Filtered distance*/
 float confidence[5]; /*< Filtered confidence*/
 float velocity[5]; /*< Filtered velocity*/
}) mavlink_radar_filtered_t;

#define MAVLINK_MSG_ID_RADAR_FILTERED_LEN 60
#define MAVLINK_MSG_ID_RADAR_FILTERED_MIN_LEN 60
#define MAVLINK_MSG_ID_181_LEN 60
#define MAVLINK_MSG_ID_181_MIN_LEN 60

#define MAVLINK_MSG_ID_RADAR_FILTERED_CRC 221
#define MAVLINK_MSG_ID_181_CRC 221

#define MAVLINK_MSG_RADAR_FILTERED_FIELD_DISTANCE_LEN 5
#define MAVLINK_MSG_RADAR_FILTERED_FIELD_CONFIDENCE_LEN 5
#define MAVLINK_MSG_RADAR_FILTERED_FIELD_VELOCITY_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RADAR_FILTERED { \
    181, \
    "RADAR_FILTERED", \
    3, \
    {  { "distance", NULL, MAVLINK_TYPE_FLOAT, 5, 0, offsetof(mavlink_radar_filtered_t, distance) }, \
         { "confidence", NULL, MAVLINK_TYPE_FLOAT, 5, 20, offsetof(mavlink_radar_filtered_t, confidence) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 5, 40, offsetof(mavlink_radar_filtered_t, velocity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RADAR_FILTERED { \
    "RADAR_FILTERED", \
    3, \
    {  { "distance", NULL, MAVLINK_TYPE_FLOAT, 5, 0, offsetof(mavlink_radar_filtered_t, distance) }, \
         { "confidence", NULL, MAVLINK_TYPE_FLOAT, 5, 20, offsetof(mavlink_radar_filtered_t, confidence) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 5, 40, offsetof(mavlink_radar_filtered_t, velocity) }, \
         } \
}
#endif

/**
 * @brief Pack a radar_filtered message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param distance Filtered distance
 * @param confidence Filtered confidence
 * @param velocity Filtered velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_filtered_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *distance, const float *confidence, const float *velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_FILTERED_LEN];

    _mav_put_float_array(buf, 0, distance, 5);
    _mav_put_float_array(buf, 20, confidence, 5);
    _mav_put_float_array(buf, 40, velocity, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_FILTERED_LEN);
#else
    mavlink_radar_filtered_t packet;

    mav_array_memcpy(packet.distance, distance, sizeof(float)*5);
    mav_array_memcpy(packet.confidence, confidence, sizeof(float)*5);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_FILTERED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADAR_FILTERED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADAR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_CRC);
}

/**
 * @brief Pack a radar_filtered message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param distance Filtered distance
 * @param confidence Filtered confidence
 * @param velocity Filtered velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_filtered_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *distance,const float *confidence,const float *velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_FILTERED_LEN];

    _mav_put_float_array(buf, 0, distance, 5);
    _mav_put_float_array(buf, 20, confidence, 5);
    _mav_put_float_array(buf, 40, velocity, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_FILTERED_LEN);
#else
    mavlink_radar_filtered_t packet;

    mav_array_memcpy(packet.distance, distance, sizeof(float)*5);
    mav_array_memcpy(packet.confidence, confidence, sizeof(float)*5);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_FILTERED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADAR_FILTERED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADAR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_CRC);
}

/**
 * @brief Encode a radar_filtered struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radar_filtered C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_filtered_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radar_filtered_t* radar_filtered)
{
    return mavlink_msg_radar_filtered_pack(system_id, component_id, msg, radar_filtered->distance, radar_filtered->confidence, radar_filtered->velocity);
}

/**
 * @brief Encode a radar_filtered struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radar_filtered C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_filtered_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radar_filtered_t* radar_filtered)
{
    return mavlink_msg_radar_filtered_pack_chan(system_id, component_id, chan, msg, radar_filtered->distance, radar_filtered->confidence, radar_filtered->velocity);
}

/**
 * @brief Send a radar_filtered message
 * @param chan MAVLink channel to send the message
 *
 * @param distance Filtered distance
 * @param confidence Filtered confidence
 * @param velocity Filtered velocity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radar_filtered_send(mavlink_channel_t chan, const float *distance, const float *confidence, const float *velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_FILTERED_LEN];

    _mav_put_float_array(buf, 0, distance, 5);
    _mav_put_float_array(buf, 20, confidence, 5);
    _mav_put_float_array(buf, 40, velocity, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_FILTERED, buf, MAVLINK_MSG_ID_RADAR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_CRC);
#else
    mavlink_radar_filtered_t packet;

    mav_array_memcpy(packet.distance, distance, sizeof(float)*5);
    mav_array_memcpy(packet.confidence, confidence, sizeof(float)*5);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_FILTERED, (const char *)&packet, MAVLINK_MSG_ID_RADAR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_CRC);
#endif
}

/**
 * @brief Send a radar_filtered message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_radar_filtered_send_struct(mavlink_channel_t chan, const mavlink_radar_filtered_t* radar_filtered)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radar_filtered_send(chan, radar_filtered->distance, radar_filtered->confidence, radar_filtered->velocity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_FILTERED, (const char *)radar_filtered, MAVLINK_MSG_ID_RADAR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_CRC);
#endif
}

#if MAVLINK_MSG_ID_RADAR_FILTERED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radar_filtered_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *distance, const float *confidence, const float *velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_float_array(buf, 0, distance, 5);
    _mav_put_float_array(buf, 20, confidence, 5);
    _mav_put_float_array(buf, 40, velocity, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_FILTERED, buf, MAVLINK_MSG_ID_RADAR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_CRC);
#else
    mavlink_radar_filtered_t *packet = (mavlink_radar_filtered_t *)msgbuf;

    mav_array_memcpy(packet->distance, distance, sizeof(float)*5);
    mav_array_memcpy(packet->confidence, confidence, sizeof(float)*5);
    mav_array_memcpy(packet->velocity, velocity, sizeof(float)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_FILTERED, (const char *)packet, MAVLINK_MSG_ID_RADAR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_LEN, MAVLINK_MSG_ID_RADAR_FILTERED_CRC);
#endif
}
#endif

#endif

// MESSAGE RADAR_FILTERED UNPACKING


/**
 * @brief Get field distance from radar_filtered message
 *
 * @return Filtered distance
 */
static inline uint16_t mavlink_msg_radar_filtered_get_distance(const mavlink_message_t* msg, float *distance)
{
    return _MAV_RETURN_float_array(msg, distance, 5,  0);
}

/**
 * @brief Get field confidence from radar_filtered message
 *
 * @return Filtered confidence
 */
static inline uint16_t mavlink_msg_radar_filtered_get_confidence(const mavlink_message_t* msg, float *confidence)
{
    return _MAV_RETURN_float_array(msg, confidence, 5,  20);
}

/**
 * @brief Get field velocity from radar_filtered message
 *
 * @return Filtered velocity
 */
static inline uint16_t mavlink_msg_radar_filtered_get_velocity(const mavlink_message_t* msg, float *velocity)
{
    return _MAV_RETURN_float_array(msg, velocity, 5,  40);
}

/**
 * @brief Decode a radar_filtered message into a struct
 *
 * @param msg The message to decode
 * @param radar_filtered C-struct to decode the message contents into
 */
static inline void mavlink_msg_radar_filtered_decode(const mavlink_message_t* msg, mavlink_radar_filtered_t* radar_filtered)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radar_filtered_get_distance(msg, radar_filtered->distance);
    mavlink_msg_radar_filtered_get_confidence(msg, radar_filtered->confidence);
    mavlink_msg_radar_filtered_get_velocity(msg, radar_filtered->velocity);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RADAR_FILTERED_LEN? msg->len : MAVLINK_MSG_ID_RADAR_FILTERED_LEN;
        memset(radar_filtered, 0, MAVLINK_MSG_ID_RADAR_FILTERED_LEN);
    memcpy(radar_filtered, _MAV_PAYLOAD(msg), len);
#endif
}
