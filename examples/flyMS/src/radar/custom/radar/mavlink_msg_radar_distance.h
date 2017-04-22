#pragma once
// MESSAGE RADAR_DISTANCE PACKING

#define MAVLINK_MSG_ID_RADAR_DISTANCE 202

MAVPACKED(
typedef struct __mavlink_radar_distance_t {
 uint16_t radar_range[5]; /*< uint16_t_array*/
 uint8_t upsweep_mag[5]; /*< uint8_t_array*/
 uint8_t downsweep_mag[5]; /*< uint8_t_array*/
}) mavlink_radar_distance_t;

#define MAVLINK_MSG_ID_RADAR_DISTANCE_LEN 20
#define MAVLINK_MSG_ID_RADAR_DISTANCE_MIN_LEN 20
#define MAVLINK_MSG_ID_202_LEN 20
#define MAVLINK_MSG_ID_202_MIN_LEN 20

#define MAVLINK_MSG_ID_RADAR_DISTANCE_CRC 175
#define MAVLINK_MSG_ID_202_CRC 175

#define MAVLINK_MSG_RADAR_DISTANCE_FIELD_RADAR_RANGE_LEN 5
#define MAVLINK_MSG_RADAR_DISTANCE_FIELD_UPSWEEP_MAG_LEN 5
#define MAVLINK_MSG_RADAR_DISTANCE_FIELD_DOWNSWEEP_MAG_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RADAR_DISTANCE { \
    202, \
    "RADAR_DISTANCE", \
    3, \
    {  { "radar_range", NULL, MAVLINK_TYPE_UINT16_T, 5, 0, offsetof(mavlink_radar_distance_t, radar_range) }, \
         { "upsweep_mag", NULL, MAVLINK_TYPE_UINT8_T, 5, 10, offsetof(mavlink_radar_distance_t, upsweep_mag) }, \
         { "downsweep_mag", NULL, MAVLINK_TYPE_UINT8_T, 5, 15, offsetof(mavlink_radar_distance_t, downsweep_mag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RADAR_DISTANCE { \
    "RADAR_DISTANCE", \
    3, \
    {  { "radar_range", NULL, MAVLINK_TYPE_UINT16_T, 5, 0, offsetof(mavlink_radar_distance_t, radar_range) }, \
         { "upsweep_mag", NULL, MAVLINK_TYPE_UINT8_T, 5, 10, offsetof(mavlink_radar_distance_t, upsweep_mag) }, \
         { "downsweep_mag", NULL, MAVLINK_TYPE_UINT8_T, 5, 15, offsetof(mavlink_radar_distance_t, downsweep_mag) }, \
         } \
}
#endif

/**
 * @brief Pack a radar_distance message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param radar_range uint16_t_array
 * @param upsweep_mag uint8_t_array
 * @param downsweep_mag uint8_t_array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_distance_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint16_t *radar_range, const uint8_t *upsweep_mag, const uint8_t *downsweep_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_DISTANCE_LEN];

    _mav_put_uint16_t_array(buf, 0, radar_range, 5);
    _mav_put_uint8_t_array(buf, 10, upsweep_mag, 5);
    _mav_put_uint8_t_array(buf, 15, downsweep_mag, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN);
#else
    mavlink_radar_distance_t packet;

    mav_array_memcpy(packet.radar_range, radar_range, sizeof(uint16_t)*5);
    mav_array_memcpy(packet.upsweep_mag, upsweep_mag, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.downsweep_mag, downsweep_mag, sizeof(uint8_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADAR_DISTANCE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADAR_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_CRC);
}

/**
 * @brief Pack a radar_distance message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radar_range uint16_t_array
 * @param upsweep_mag uint8_t_array
 * @param downsweep_mag uint8_t_array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_distance_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint16_t *radar_range,const uint8_t *upsweep_mag,const uint8_t *downsweep_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_DISTANCE_LEN];

    _mav_put_uint16_t_array(buf, 0, radar_range, 5);
    _mav_put_uint8_t_array(buf, 10, upsweep_mag, 5);
    _mav_put_uint8_t_array(buf, 15, downsweep_mag, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN);
#else
    mavlink_radar_distance_t packet;

    mav_array_memcpy(packet.radar_range, radar_range, sizeof(uint16_t)*5);
    mav_array_memcpy(packet.upsweep_mag, upsweep_mag, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.downsweep_mag, downsweep_mag, sizeof(uint8_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADAR_DISTANCE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADAR_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_CRC);
}

/**
 * @brief Encode a radar_distance struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radar_distance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_distance_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radar_distance_t* radar_distance)
{
    return mavlink_msg_radar_distance_pack(system_id, component_id, msg, radar_distance->radar_range, radar_distance->upsweep_mag, radar_distance->downsweep_mag);
}

/**
 * @brief Encode a radar_distance struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radar_distance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_distance_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radar_distance_t* radar_distance)
{
    return mavlink_msg_radar_distance_pack_chan(system_id, component_id, chan, msg, radar_distance->radar_range, radar_distance->upsweep_mag, radar_distance->downsweep_mag);
}

/**
 * @brief Send a radar_distance message
 * @param chan MAVLink channel to send the message
 *
 * @param radar_range uint16_t_array
 * @param upsweep_mag uint8_t_array
 * @param downsweep_mag uint8_t_array
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radar_distance_send(mavlink_channel_t chan, const uint16_t *radar_range, const uint8_t *upsweep_mag, const uint8_t *downsweep_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_DISTANCE_LEN];

    _mav_put_uint16_t_array(buf, 0, radar_range, 5);
    _mav_put_uint8_t_array(buf, 10, upsweep_mag, 5);
    _mav_put_uint8_t_array(buf, 15, downsweep_mag, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_DISTANCE, buf, MAVLINK_MSG_ID_RADAR_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_CRC);
#else
    mavlink_radar_distance_t packet;

    mav_array_memcpy(packet.radar_range, radar_range, sizeof(uint16_t)*5);
    mav_array_memcpy(packet.upsweep_mag, upsweep_mag, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.downsweep_mag, downsweep_mag, sizeof(uint8_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_DISTANCE, (const char *)&packet, MAVLINK_MSG_ID_RADAR_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_CRC);
#endif
}

/**
 * @brief Send a radar_distance message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_radar_distance_send_struct(mavlink_channel_t chan, const mavlink_radar_distance_t* radar_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radar_distance_send(chan, radar_distance->radar_range, radar_distance->upsweep_mag, radar_distance->downsweep_mag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_DISTANCE, (const char *)radar_distance, MAVLINK_MSG_ID_RADAR_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_CRC);
#endif
}

#if MAVLINK_MSG_ID_RADAR_DISTANCE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radar_distance_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint16_t *radar_range, const uint8_t *upsweep_mag, const uint8_t *downsweep_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint16_t_array(buf, 0, radar_range, 5);
    _mav_put_uint8_t_array(buf, 10, upsweep_mag, 5);
    _mav_put_uint8_t_array(buf, 15, downsweep_mag, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_DISTANCE, buf, MAVLINK_MSG_ID_RADAR_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_CRC);
#else
    mavlink_radar_distance_t *packet = (mavlink_radar_distance_t *)msgbuf;

    mav_array_memcpy(packet->radar_range, radar_range, sizeof(uint16_t)*5);
    mav_array_memcpy(packet->upsweep_mag, upsweep_mag, sizeof(uint8_t)*5);
    mav_array_memcpy(packet->downsweep_mag, downsweep_mag, sizeof(uint8_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_DISTANCE, (const char *)packet, MAVLINK_MSG_ID_RADAR_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN, MAVLINK_MSG_ID_RADAR_DISTANCE_CRC);
#endif
}
#endif

#endif

// MESSAGE RADAR_DISTANCE UNPACKING


/**
 * @brief Get field radar_range from radar_distance message
 *
 * @return uint16_t_array
 */
static inline uint16_t mavlink_msg_radar_distance_get_radar_range(const mavlink_message_t* msg, uint16_t *radar_range)
{
    return _MAV_RETURN_uint16_t_array(msg, radar_range, 5,  0);
}

/**
 * @brief Get field upsweep_mag from radar_distance message
 *
 * @return uint8_t_array
 */
static inline uint16_t mavlink_msg_radar_distance_get_upsweep_mag(const mavlink_message_t* msg, uint8_t *upsweep_mag)
{
    return _MAV_RETURN_uint8_t_array(msg, upsweep_mag, 5,  10);
}

/**
 * @brief Get field downsweep_mag from radar_distance message
 *
 * @return uint8_t_array
 */
static inline uint16_t mavlink_msg_radar_distance_get_downsweep_mag(const mavlink_message_t* msg, uint8_t *downsweep_mag)
{
    return _MAV_RETURN_uint8_t_array(msg, downsweep_mag, 5,  15);
}

/**
 * @brief Decode a radar_distance message into a struct
 *
 * @param msg The message to decode
 * @param radar_distance C-struct to decode the message contents into
 */
static inline void mavlink_msg_radar_distance_decode(const mavlink_message_t* msg, mavlink_radar_distance_t* radar_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radar_distance_get_radar_range(msg, radar_distance->radar_range);
    mavlink_msg_radar_distance_get_upsweep_mag(msg, radar_distance->upsweep_mag);
    mavlink_msg_radar_distance_get_downsweep_mag(msg, radar_distance->downsweep_mag);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RADAR_DISTANCE_LEN? msg->len : MAVLINK_MSG_ID_RADAR_DISTANCE_LEN;
        memset(radar_distance, 0, MAVLINK_MSG_ID_RADAR_DISTANCE_LEN);
    memcpy(radar_distance, _MAV_PAYLOAD(msg), len);
#endif
}
