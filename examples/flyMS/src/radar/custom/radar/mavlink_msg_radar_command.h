#pragma once
// MESSAGE RADAR_COMMAND PACKING

#define MAVLINK_MSG_ID_RADAR_COMMAND 201

MAVPACKED(
typedef struct __mavlink_radar_command_t {
 uint32_t address; /*< uint32_t*/
 uint32_t data; /*< uint32_t*/
 uint8_t command; /*< uint8_t*/
}) mavlink_radar_command_t;

#define MAVLINK_MSG_ID_RADAR_COMMAND_LEN 9
#define MAVLINK_MSG_ID_RADAR_COMMAND_MIN_LEN 9
#define MAVLINK_MSG_ID_201_LEN 9
#define MAVLINK_MSG_ID_201_MIN_LEN 9

#define MAVLINK_MSG_ID_RADAR_COMMAND_CRC 117
#define MAVLINK_MSG_ID_201_CRC 117



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RADAR_COMMAND { \
    201, \
    "RADAR_COMMAND", \
    3, \
    {  { "address", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radar_command_t, address) }, \
         { "data", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_radar_command_t, data) }, \
         { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_radar_command_t, command) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RADAR_COMMAND { \
    "RADAR_COMMAND", \
    3, \
    {  { "address", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radar_command_t, address) }, \
         { "data", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_radar_command_t, data) }, \
         { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_radar_command_t, command) }, \
         } \
}
#endif

/**
 * @brief Pack a radar_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command uint8_t
 * @param address uint32_t
 * @param data uint32_t
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t command, uint32_t address, uint32_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, address);
    _mav_put_uint32_t(buf, 4, data);
    _mav_put_uint8_t(buf, 8, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_COMMAND_LEN);
#else
    mavlink_radar_command_t packet;
    packet.address = address;
    packet.data = data;
    packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADAR_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADAR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_CRC);
}

/**
 * @brief Pack a radar_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command uint8_t
 * @param address uint32_t
 * @param data uint32_t
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t command,uint32_t address,uint32_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, address);
    _mav_put_uint32_t(buf, 4, data);
    _mav_put_uint8_t(buf, 8, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_COMMAND_LEN);
#else
    mavlink_radar_command_t packet;
    packet.address = address;
    packet.data = data;
    packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADAR_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADAR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_CRC);
}

/**
 * @brief Encode a radar_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radar_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radar_command_t* radar_command)
{
    return mavlink_msg_radar_command_pack(system_id, component_id, msg, radar_command->command, radar_command->address, radar_command->data);
}

/**
 * @brief Encode a radar_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radar_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radar_command_t* radar_command)
{
    return mavlink_msg_radar_command_pack_chan(system_id, component_id, chan, msg, radar_command->command, radar_command->address, radar_command->data);
}

/**
 * @brief Send a radar_command message
 * @param chan MAVLink channel to send the message
 *
 * @param command uint8_t
 * @param address uint32_t
 * @param data uint32_t
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radar_command_send(mavlink_channel_t chan, uint8_t command, uint32_t address, uint32_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, address);
    _mav_put_uint32_t(buf, 4, data);
    _mav_put_uint8_t(buf, 8, command);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_COMMAND, buf, MAVLINK_MSG_ID_RADAR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_CRC);
#else
    mavlink_radar_command_t packet;
    packet.address = address;
    packet.data = data;
    packet.command = command;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_RADAR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_CRC);
#endif
}

/**
 * @brief Send a radar_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_radar_command_send_struct(mavlink_channel_t chan, const mavlink_radar_command_t* radar_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radar_command_send(chan, radar_command->command, radar_command->address, radar_command->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_COMMAND, (const char *)radar_command, MAVLINK_MSG_ID_RADAR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_RADAR_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radar_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t command, uint32_t address, uint32_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, address);
    _mav_put_uint32_t(buf, 4, data);
    _mav_put_uint8_t(buf, 8, command);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_COMMAND, buf, MAVLINK_MSG_ID_RADAR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_CRC);
#else
    mavlink_radar_command_t *packet = (mavlink_radar_command_t *)msgbuf;
    packet->address = address;
    packet->data = data;
    packet->command = command;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_COMMAND, (const char *)packet, MAVLINK_MSG_ID_RADAR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_LEN, MAVLINK_MSG_ID_RADAR_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE RADAR_COMMAND UNPACKING


/**
 * @brief Get field command from radar_command message
 *
 * @return uint8_t
 */
static inline uint8_t mavlink_msg_radar_command_get_command(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field address from radar_command message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_command_get_address(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field data from radar_command message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_command_get_data(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a radar_command message into a struct
 *
 * @param msg The message to decode
 * @param radar_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_radar_command_decode(const mavlink_message_t* msg, mavlink_radar_command_t* radar_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    radar_command->address = mavlink_msg_radar_command_get_address(msg);
    radar_command->data = mavlink_msg_radar_command_get_data(msg);
    radar_command->command = mavlink_msg_radar_command_get_command(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RADAR_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_RADAR_COMMAND_LEN;
        memset(radar_command, 0, MAVLINK_MSG_ID_RADAR_COMMAND_LEN);
    memcpy(radar_command, _MAV_PAYLOAD(msg), len);
#endif
}
