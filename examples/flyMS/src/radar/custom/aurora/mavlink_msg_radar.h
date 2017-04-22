#pragma once
// MESSAGE RADAR PACKING

#define MAVLINK_MSG_ID_RADAR 180

MAVPACKED(
typedef struct __mavlink_radar_t {
 int32_t n; /*< Radar sensor number of elements*/
 uint16_t distance[5]; /*< Radar sensor distance*/
 uint8_t up_mag[5]; /*< Radar sensor up magnitude*/
 uint8_t down_mag[5]; /*< Radar sensor down magnitude*/
}) mavlink_radar_t;

#define MAVLINK_MSG_ID_RADAR_LEN 24
#define MAVLINK_MSG_ID_RADAR_MIN_LEN 24
#define MAVLINK_MSG_ID_180_LEN 24
#define MAVLINK_MSG_ID_180_MIN_LEN 24

#define MAVLINK_MSG_ID_RADAR_CRC 211
#define MAVLINK_MSG_ID_180_CRC 211

#define MAVLINK_MSG_RADAR_FIELD_DISTANCE_LEN 5
#define MAVLINK_MSG_RADAR_FIELD_UP_MAG_LEN 5
#define MAVLINK_MSG_RADAR_FIELD_DOWN_MAG_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RADAR { \
    180, \
    "RADAR", \
    4, \
    {  { "n", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_radar_t, n) }, \
         { "distance", NULL, MAVLINK_TYPE_UINT16_T, 5, 4, offsetof(mavlink_radar_t, distance) }, \
         { "up_mag", NULL, MAVLINK_TYPE_UINT8_T, 5, 14, offsetof(mavlink_radar_t, up_mag) }, \
         { "down_mag", NULL, MAVLINK_TYPE_UINT8_T, 5, 19, offsetof(mavlink_radar_t, down_mag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RADAR { \
    "RADAR", \
    4, \
    {  { "n", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_radar_t, n) }, \
         { "distance", NULL, MAVLINK_TYPE_UINT16_T, 5, 4, offsetof(mavlink_radar_t, distance) }, \
         { "up_mag", NULL, MAVLINK_TYPE_UINT8_T, 5, 14, offsetof(mavlink_radar_t, up_mag) }, \
         { "down_mag", NULL, MAVLINK_TYPE_UINT8_T, 5, 19, offsetof(mavlink_radar_t, down_mag) }, \
         } \
}
#endif

/**
 * @brief Pack a radar message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param n Radar sensor number of elements
 * @param distance Radar sensor distance
 * @param up_mag Radar sensor up magnitude
 * @param down_mag Radar sensor down magnitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t n, const uint16_t *distance, const uint8_t *up_mag, const uint8_t *down_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_LEN];
    _mav_put_int32_t(buf, 0, n);
    _mav_put_uint16_t_array(buf, 4, distance, 5);
    _mav_put_uint8_t_array(buf, 14, up_mag, 5);
    _mav_put_uint8_t_array(buf, 19, down_mag, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_LEN);
#else
    mavlink_radar_t packet;
    packet.n = n;
    mav_array_memcpy(packet.distance, distance, sizeof(uint16_t)*5);
    mav_array_memcpy(packet.up_mag, up_mag, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.down_mag, down_mag, sizeof(uint8_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADAR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADAR_MIN_LEN, MAVLINK_MSG_ID_RADAR_LEN, MAVLINK_MSG_ID_RADAR_CRC);
}

/**
 * @brief Pack a radar message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param n Radar sensor number of elements
 * @param distance Radar sensor distance
 * @param up_mag Radar sensor up magnitude
 * @param down_mag Radar sensor down magnitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t n,const uint16_t *distance,const uint8_t *up_mag,const uint8_t *down_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_LEN];
    _mav_put_int32_t(buf, 0, n);
    _mav_put_uint16_t_array(buf, 4, distance, 5);
    _mav_put_uint8_t_array(buf, 14, up_mag, 5);
    _mav_put_uint8_t_array(buf, 19, down_mag, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_LEN);
#else
    mavlink_radar_t packet;
    packet.n = n;
    mav_array_memcpy(packet.distance, distance, sizeof(uint16_t)*5);
    mav_array_memcpy(packet.up_mag, up_mag, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.down_mag, down_mag, sizeof(uint8_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADAR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADAR_MIN_LEN, MAVLINK_MSG_ID_RADAR_LEN, MAVLINK_MSG_ID_RADAR_CRC);
}

/**
 * @brief Encode a radar struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radar C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radar_t* radar)
{
    return mavlink_msg_radar_pack(system_id, component_id, msg, radar->n, radar->distance, radar->up_mag, radar->down_mag);
}

/**
 * @brief Encode a radar struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radar C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radar_t* radar)
{
    return mavlink_msg_radar_pack_chan(system_id, component_id, chan, msg, radar->n, radar->distance, radar->up_mag, radar->down_mag);
}

/**
 * @brief Send a radar message
 * @param chan MAVLink channel to send the message
 *
 * @param n Radar sensor number of elements
 * @param distance Radar sensor distance
 * @param up_mag Radar sensor up magnitude
 * @param down_mag Radar sensor down magnitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radar_send(mavlink_channel_t chan, int32_t n, const uint16_t *distance, const uint8_t *up_mag, const uint8_t *down_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_LEN];
    _mav_put_int32_t(buf, 0, n);
    _mav_put_uint16_t_array(buf, 4, distance, 5);
    _mav_put_uint8_t_array(buf, 14, up_mag, 5);
    _mav_put_uint8_t_array(buf, 19, down_mag, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR, buf, MAVLINK_MSG_ID_RADAR_MIN_LEN, MAVLINK_MSG_ID_RADAR_LEN, MAVLINK_MSG_ID_RADAR_CRC);
#else
    mavlink_radar_t packet;
    packet.n = n;
    mav_array_memcpy(packet.distance, distance, sizeof(uint16_t)*5);
    mav_array_memcpy(packet.up_mag, up_mag, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.down_mag, down_mag, sizeof(uint8_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR, (const char *)&packet, MAVLINK_MSG_ID_RADAR_MIN_LEN, MAVLINK_MSG_ID_RADAR_LEN, MAVLINK_MSG_ID_RADAR_CRC);
#endif
}

/**
 * @brief Send a radar message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_radar_send_struct(mavlink_channel_t chan, const mavlink_radar_t* radar)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radar_send(chan, radar->n, radar->distance, radar->up_mag, radar->down_mag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR, (const char *)radar, MAVLINK_MSG_ID_RADAR_MIN_LEN, MAVLINK_MSG_ID_RADAR_LEN, MAVLINK_MSG_ID_RADAR_CRC);
#endif
}

#if MAVLINK_MSG_ID_RADAR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radar_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t n, const uint16_t *distance, const uint8_t *up_mag, const uint8_t *down_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, n);
    _mav_put_uint16_t_array(buf, 4, distance, 5);
    _mav_put_uint8_t_array(buf, 14, up_mag, 5);
    _mav_put_uint8_t_array(buf, 19, down_mag, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR, buf, MAVLINK_MSG_ID_RADAR_MIN_LEN, MAVLINK_MSG_ID_RADAR_LEN, MAVLINK_MSG_ID_RADAR_CRC);
#else
    mavlink_radar_t *packet = (mavlink_radar_t *)msgbuf;
    packet->n = n;
    mav_array_memcpy(packet->distance, distance, sizeof(uint16_t)*5);
    mav_array_memcpy(packet->up_mag, up_mag, sizeof(uint8_t)*5);
    mav_array_memcpy(packet->down_mag, down_mag, sizeof(uint8_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR, (const char *)packet, MAVLINK_MSG_ID_RADAR_MIN_LEN, MAVLINK_MSG_ID_RADAR_LEN, MAVLINK_MSG_ID_RADAR_CRC);
#endif
}
#endif

#endif

// MESSAGE RADAR UNPACKING


/**
 * @brief Get field n from radar message
 *
 * @return Radar sensor number of elements
 */
static inline int32_t mavlink_msg_radar_get_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field distance from radar message
 *
 * @return Radar sensor distance
 */
static inline uint16_t mavlink_msg_radar_get_distance(const mavlink_message_t* msg, uint16_t *distance)
{
    return _MAV_RETURN_uint16_t_array(msg, distance, 5,  4);
}

/**
 * @brief Get field up_mag from radar message
 *
 * @return Radar sensor up magnitude
 */
static inline uint16_t mavlink_msg_radar_get_up_mag(const mavlink_message_t* msg, uint8_t *up_mag)
{
    return _MAV_RETURN_uint8_t_array(msg, up_mag, 5,  14);
}

/**
 * @brief Get field down_mag from radar message
 *
 * @return Radar sensor down magnitude
 */
static inline uint16_t mavlink_msg_radar_get_down_mag(const mavlink_message_t* msg, uint8_t *down_mag)
{
    return _MAV_RETURN_uint8_t_array(msg, down_mag, 5,  19);
}

/**
 * @brief Decode a radar message into a struct
 *
 * @param msg The message to decode
 * @param radar C-struct to decode the message contents into
 */
static inline void mavlink_msg_radar_decode(const mavlink_message_t* msg, mavlink_radar_t* radar)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    radar->n = mavlink_msg_radar_get_n(msg);
    mavlink_msg_radar_get_distance(msg, radar->distance);
    mavlink_msg_radar_get_up_mag(msg, radar->up_mag);
    mavlink_msg_radar_get_down_mag(msg, radar->down_mag);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RADAR_LEN? msg->len : MAVLINK_MSG_ID_RADAR_LEN;
        memset(radar, 0, MAVLINK_MSG_ID_RADAR_LEN);
    memcpy(radar, _MAV_PAYLOAD(msg), len);
#endif
}
