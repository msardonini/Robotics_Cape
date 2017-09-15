#pragma once
// MESSAGE RADAR_PARAM PACKING

#define MAVLINK_MSG_ID_RADAR_PARAM 200

MAVPACKED(
typedef struct __mavlink_radar_param_t {
 uint32_t sensing_mode; /*< uint32_t*/
 uint32_t data_sel; /*< uint32_t*/
 uint32_t decimate; /*< uint32_t*/
 uint32_t rx_gain; /*< uint32_t*/
 float fout_ch1; /*< float*/
 uint32_t fmcw_t; /*< uint32_t*/
 float fmcw_f; /*< float*/
 float alpha; /*< float*/
 uint32_t weight; /*< uint32_t*/
 float thrsh_db; /*< float*/
 uint32_t cal_period; /*< uint32_t*/
 float lower; /*< float*/
 float upper; /*< float*/
 uint32_t search; /*< uint32_t*/
 uint32_t window_func; /*< uint32_t*/
 uint32_t dc_cut; /*< uint32_t*/
 uint32_t output; /*< uint32_t*/
 uint32_t nfft; /*< uint32_t*/
 float period; /*< float*/
 uint8_t fmcw_fast; /*< uint8_t*/
 uint8_t enlpf; /*< uint8_t*/
}) mavlink_radar_param_t;

#define MAVLINK_MSG_ID_RADAR_PARAM_LEN 78
#define MAVLINK_MSG_ID_RADAR_PARAM_MIN_LEN 78
#define MAVLINK_MSG_ID_200_LEN 78
#define MAVLINK_MSG_ID_200_MIN_LEN 78

#define MAVLINK_MSG_ID_RADAR_PARAM_CRC 172
#define MAVLINK_MSG_ID_200_CRC 172



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RADAR_PARAM { \
    200, \
    "RADAR_PARAM", \
    21, \
    {  { "sensing_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radar_param_t, sensing_mode) }, \
         { "data_sel", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_radar_param_t, data_sel) }, \
         { "decimate", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_radar_param_t, decimate) }, \
         { "rx_gain", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_radar_param_t, rx_gain) }, \
         { "fout_ch1", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_radar_param_t, fout_ch1) }, \
         { "fmcw_t", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_radar_param_t, fmcw_t) }, \
         { "fmcw_f", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_radar_param_t, fmcw_f) }, \
         { "alpha", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_radar_param_t, alpha) }, \
         { "weight", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_radar_param_t, weight) }, \
         { "thrsh_db", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_radar_param_t, thrsh_db) }, \
         { "cal_period", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_radar_param_t, cal_period) }, \
         { "lower", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_radar_param_t, lower) }, \
         { "upper", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_radar_param_t, upper) }, \
         { "search", NULL, MAVLINK_TYPE_UINT32_T, 0, 52, offsetof(mavlink_radar_param_t, search) }, \
         { "window_func", NULL, MAVLINK_TYPE_UINT32_T, 0, 56, offsetof(mavlink_radar_param_t, window_func) }, \
         { "dc_cut", NULL, MAVLINK_TYPE_UINT32_T, 0, 60, offsetof(mavlink_radar_param_t, dc_cut) }, \
         { "output", NULL, MAVLINK_TYPE_UINT32_T, 0, 64, offsetof(mavlink_radar_param_t, output) }, \
         { "nfft", NULL, MAVLINK_TYPE_UINT32_T, 0, 68, offsetof(mavlink_radar_param_t, nfft) }, \
         { "period", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_radar_param_t, period) }, \
         { "fmcw_fast", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_radar_param_t, fmcw_fast) }, \
         { "enlpf", NULL, MAVLINK_TYPE_UINT8_T, 0, 77, offsetof(mavlink_radar_param_t, enlpf) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RADAR_PARAM { \
    "RADAR_PARAM", \
    21, \
    {  { "sensing_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radar_param_t, sensing_mode) }, \
         { "data_sel", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_radar_param_t, data_sel) }, \
         { "decimate", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_radar_param_t, decimate) }, \
         { "rx_gain", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_radar_param_t, rx_gain) }, \
         { "fout_ch1", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_radar_param_t, fout_ch1) }, \
         { "fmcw_t", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_radar_param_t, fmcw_t) }, \
         { "fmcw_f", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_radar_param_t, fmcw_f) }, \
         { "alpha", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_radar_param_t, alpha) }, \
         { "weight", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_radar_param_t, weight) }, \
         { "thrsh_db", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_radar_param_t, thrsh_db) }, \
         { "cal_period", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_radar_param_t, cal_period) }, \
         { "lower", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_radar_param_t, lower) }, \
         { "upper", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_radar_param_t, upper) }, \
         { "search", NULL, MAVLINK_TYPE_UINT32_T, 0, 52, offsetof(mavlink_radar_param_t, search) }, \
         { "window_func", NULL, MAVLINK_TYPE_UINT32_T, 0, 56, offsetof(mavlink_radar_param_t, window_func) }, \
         { "dc_cut", NULL, MAVLINK_TYPE_UINT32_T, 0, 60, offsetof(mavlink_radar_param_t, dc_cut) }, \
         { "output", NULL, MAVLINK_TYPE_UINT32_T, 0, 64, offsetof(mavlink_radar_param_t, output) }, \
         { "nfft", NULL, MAVLINK_TYPE_UINT32_T, 0, 68, offsetof(mavlink_radar_param_t, nfft) }, \
         { "period", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_radar_param_t, period) }, \
         { "fmcw_fast", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_radar_param_t, fmcw_fast) }, \
         { "enlpf", NULL, MAVLINK_TYPE_UINT8_T, 0, 77, offsetof(mavlink_radar_param_t, enlpf) }, \
         } \
}
#endif

/**
 * @brief Pack a radar_param message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sensing_mode uint32_t
 * @param data_sel uint32_t
 * @param decimate uint32_t
 * @param rx_gain uint32_t
 * @param fout_ch1 float
 * @param fmcw_t uint32_t
 * @param fmcw_f float
 * @param fmcw_fast uint8_t
 * @param alpha float
 * @param weight uint32_t
 * @param thrsh_db float
 * @param cal_period uint32_t
 * @param lower float
 * @param upper float
 * @param search uint32_t
 * @param window_func uint32_t
 * @param dc_cut uint32_t
 * @param output uint32_t
 * @param nfft uint32_t
 * @param period float
 * @param enlpf uint8_t
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_param_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t sensing_mode, uint32_t data_sel, uint32_t decimate, uint32_t rx_gain, float fout_ch1, uint32_t fmcw_t, float fmcw_f, uint8_t fmcw_fast, float alpha, uint32_t weight, float thrsh_db, uint32_t cal_period, float lower, float upper, uint32_t search, uint32_t window_func, uint32_t dc_cut, uint32_t output, uint32_t nfft, float period, uint8_t enlpf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_PARAM_LEN];
    _mav_put_uint32_t(buf, 0, sensing_mode);
    _mav_put_uint32_t(buf, 4, data_sel);
    _mav_put_uint32_t(buf, 8, decimate);
    _mav_put_uint32_t(buf, 12, rx_gain);
    _mav_put_float(buf, 16, fout_ch1);
    _mav_put_uint32_t(buf, 20, fmcw_t);
    _mav_put_float(buf, 24, fmcw_f);
    _mav_put_float(buf, 28, alpha);
    _mav_put_uint32_t(buf, 32, weight);
    _mav_put_float(buf, 36, thrsh_db);
    _mav_put_uint32_t(buf, 40, cal_period);
    _mav_put_float(buf, 44, lower);
    _mav_put_float(buf, 48, upper);
    _mav_put_uint32_t(buf, 52, search);
    _mav_put_uint32_t(buf, 56, window_func);
    _mav_put_uint32_t(buf, 60, dc_cut);
    _mav_put_uint32_t(buf, 64, output);
    _mav_put_uint32_t(buf, 68, nfft);
    _mav_put_float(buf, 72, period);
    _mav_put_uint8_t(buf, 76, fmcw_fast);
    _mav_put_uint8_t(buf, 77, enlpf);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_PARAM_LEN);
#else
    mavlink_radar_param_t packet;
    packet.sensing_mode = sensing_mode;
    packet.data_sel = data_sel;
    packet.decimate = decimate;
    packet.rx_gain = rx_gain;
    packet.fout_ch1 = fout_ch1;
    packet.fmcw_t = fmcw_t;
    packet.fmcw_f = fmcw_f;
    packet.alpha = alpha;
    packet.weight = weight;
    packet.thrsh_db = thrsh_db;
    packet.cal_period = cal_period;
    packet.lower = lower;
    packet.upper = upper;
    packet.search = search;
    packet.window_func = window_func;
    packet.dc_cut = dc_cut;
    packet.output = output;
    packet.nfft = nfft;
    packet.period = period;
    packet.fmcw_fast = fmcw_fast;
    packet.enlpf = enlpf;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_PARAM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADAR_PARAM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADAR_PARAM_MIN_LEN, MAVLINK_MSG_ID_RADAR_PARAM_LEN, MAVLINK_MSG_ID_RADAR_PARAM_CRC);
}

/**
 * @brief Pack a radar_param message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensing_mode uint32_t
 * @param data_sel uint32_t
 * @param decimate uint32_t
 * @param rx_gain uint32_t
 * @param fout_ch1 float
 * @param fmcw_t uint32_t
 * @param fmcw_f float
 * @param fmcw_fast uint8_t
 * @param alpha float
 * @param weight uint32_t
 * @param thrsh_db float
 * @param cal_period uint32_t
 * @param lower float
 * @param upper float
 * @param search uint32_t
 * @param window_func uint32_t
 * @param dc_cut uint32_t
 * @param output uint32_t
 * @param nfft uint32_t
 * @param period float
 * @param enlpf uint8_t
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_param_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t sensing_mode,uint32_t data_sel,uint32_t decimate,uint32_t rx_gain,float fout_ch1,uint32_t fmcw_t,float fmcw_f,uint8_t fmcw_fast,float alpha,uint32_t weight,float thrsh_db,uint32_t cal_period,float lower,float upper,uint32_t search,uint32_t window_func,uint32_t dc_cut,uint32_t output,uint32_t nfft,float period,uint8_t enlpf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_PARAM_LEN];
    _mav_put_uint32_t(buf, 0, sensing_mode);
    _mav_put_uint32_t(buf, 4, data_sel);
    _mav_put_uint32_t(buf, 8, decimate);
    _mav_put_uint32_t(buf, 12, rx_gain);
    _mav_put_float(buf, 16, fout_ch1);
    _mav_put_uint32_t(buf, 20, fmcw_t);
    _mav_put_float(buf, 24, fmcw_f);
    _mav_put_float(buf, 28, alpha);
    _mav_put_uint32_t(buf, 32, weight);
    _mav_put_float(buf, 36, thrsh_db);
    _mav_put_uint32_t(buf, 40, cal_period);
    _mav_put_float(buf, 44, lower);
    _mav_put_float(buf, 48, upper);
    _mav_put_uint32_t(buf, 52, search);
    _mav_put_uint32_t(buf, 56, window_func);
    _mav_put_uint32_t(buf, 60, dc_cut);
    _mav_put_uint32_t(buf, 64, output);
    _mav_put_uint32_t(buf, 68, nfft);
    _mav_put_float(buf, 72, period);
    _mav_put_uint8_t(buf, 76, fmcw_fast);
    _mav_put_uint8_t(buf, 77, enlpf);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_PARAM_LEN);
#else
    mavlink_radar_param_t packet;
    packet.sensing_mode = sensing_mode;
    packet.data_sel = data_sel;
    packet.decimate = decimate;
    packet.rx_gain = rx_gain;
    packet.fout_ch1 = fout_ch1;
    packet.fmcw_t = fmcw_t;
    packet.fmcw_f = fmcw_f;
    packet.alpha = alpha;
    packet.weight = weight;
    packet.thrsh_db = thrsh_db;
    packet.cal_period = cal_period;
    packet.lower = lower;
    packet.upper = upper;
    packet.search = search;
    packet.window_func = window_func;
    packet.dc_cut = dc_cut;
    packet.output = output;
    packet.nfft = nfft;
    packet.period = period;
    packet.fmcw_fast = fmcw_fast;
    packet.enlpf = enlpf;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_PARAM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADAR_PARAM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADAR_PARAM_MIN_LEN, MAVLINK_MSG_ID_RADAR_PARAM_LEN, MAVLINK_MSG_ID_RADAR_PARAM_CRC);
}

/**
 * @brief Encode a radar_param struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radar_param C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_param_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radar_param_t* radar_param)
{
    return mavlink_msg_radar_param_pack(system_id, component_id, msg, radar_param->sensing_mode, radar_param->data_sel, radar_param->decimate, radar_param->rx_gain, radar_param->fout_ch1, radar_param->fmcw_t, radar_param->fmcw_f, radar_param->fmcw_fast, radar_param->alpha, radar_param->weight, radar_param->thrsh_db, radar_param->cal_period, radar_param->lower, radar_param->upper, radar_param->search, radar_param->window_func, radar_param->dc_cut, radar_param->output, radar_param->nfft, radar_param->period, radar_param->enlpf);
}

/**
 * @brief Encode a radar_param struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radar_param C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_param_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radar_param_t* radar_param)
{
    return mavlink_msg_radar_param_pack_chan(system_id, component_id, chan, msg, radar_param->sensing_mode, radar_param->data_sel, radar_param->decimate, radar_param->rx_gain, radar_param->fout_ch1, radar_param->fmcw_t, radar_param->fmcw_f, radar_param->fmcw_fast, radar_param->alpha, radar_param->weight, radar_param->thrsh_db, radar_param->cal_period, radar_param->lower, radar_param->upper, radar_param->search, radar_param->window_func, radar_param->dc_cut, radar_param->output, radar_param->nfft, radar_param->period, radar_param->enlpf);
}

/**
 * @brief Send a radar_param message
 * @param chan MAVLink channel to send the message
 *
 * @param sensing_mode uint32_t
 * @param data_sel uint32_t
 * @param decimate uint32_t
 * @param rx_gain uint32_t
 * @param fout_ch1 float
 * @param fmcw_t uint32_t
 * @param fmcw_f float
 * @param fmcw_fast uint8_t
 * @param alpha float
 * @param weight uint32_t
 * @param thrsh_db float
 * @param cal_period uint32_t
 * @param lower float
 * @param upper float
 * @param search uint32_t
 * @param window_func uint32_t
 * @param dc_cut uint32_t
 * @param output uint32_t
 * @param nfft uint32_t
 * @param period float
 * @param enlpf uint8_t
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radar_param_send(mavlink_channel_t chan, uint32_t sensing_mode, uint32_t data_sel, uint32_t decimate, uint32_t rx_gain, float fout_ch1, uint32_t fmcw_t, float fmcw_f, uint8_t fmcw_fast, float alpha, uint32_t weight, float thrsh_db, uint32_t cal_period, float lower, float upper, uint32_t search, uint32_t window_func, uint32_t dc_cut, uint32_t output, uint32_t nfft, float period, uint8_t enlpf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADAR_PARAM_LEN];
    _mav_put_uint32_t(buf, 0, sensing_mode);
    _mav_put_uint32_t(buf, 4, data_sel);
    _mav_put_uint32_t(buf, 8, decimate);
    _mav_put_uint32_t(buf, 12, rx_gain);
    _mav_put_float(buf, 16, fout_ch1);
    _mav_put_uint32_t(buf, 20, fmcw_t);
    _mav_put_float(buf, 24, fmcw_f);
    _mav_put_float(buf, 28, alpha);
    _mav_put_uint32_t(buf, 32, weight);
    _mav_put_float(buf, 36, thrsh_db);
    _mav_put_uint32_t(buf, 40, cal_period);
    _mav_put_float(buf, 44, lower);
    _mav_put_float(buf, 48, upper);
    _mav_put_uint32_t(buf, 52, search);
    _mav_put_uint32_t(buf, 56, window_func);
    _mav_put_uint32_t(buf, 60, dc_cut);
    _mav_put_uint32_t(buf, 64, output);
    _mav_put_uint32_t(buf, 68, nfft);
    _mav_put_float(buf, 72, period);
    _mav_put_uint8_t(buf, 76, fmcw_fast);
    _mav_put_uint8_t(buf, 77, enlpf);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_PARAM, buf, MAVLINK_MSG_ID_RADAR_PARAM_MIN_LEN, MAVLINK_MSG_ID_RADAR_PARAM_LEN, MAVLINK_MSG_ID_RADAR_PARAM_CRC);
#else
    mavlink_radar_param_t packet;
    packet.sensing_mode = sensing_mode;
    packet.data_sel = data_sel;
    packet.decimate = decimate;
    packet.rx_gain = rx_gain;
    packet.fout_ch1 = fout_ch1;
    packet.fmcw_t = fmcw_t;
    packet.fmcw_f = fmcw_f;
    packet.alpha = alpha;
    packet.weight = weight;
    packet.thrsh_db = thrsh_db;
    packet.cal_period = cal_period;
    packet.lower = lower;
    packet.upper = upper;
    packet.search = search;
    packet.window_func = window_func;
    packet.dc_cut = dc_cut;
    packet.output = output;
    packet.nfft = nfft;
    packet.period = period;
    packet.fmcw_fast = fmcw_fast;
    packet.enlpf = enlpf;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_PARAM, (const char *)&packet, MAVLINK_MSG_ID_RADAR_PARAM_MIN_LEN, MAVLINK_MSG_ID_RADAR_PARAM_LEN, MAVLINK_MSG_ID_RADAR_PARAM_CRC);
#endif
}

/**
 * @brief Send a radar_param message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_radar_param_send_struct(mavlink_channel_t chan, const mavlink_radar_param_t* radar_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radar_param_send(chan, radar_param->sensing_mode, radar_param->data_sel, radar_param->decimate, radar_param->rx_gain, radar_param->fout_ch1, radar_param->fmcw_t, radar_param->fmcw_f, radar_param->fmcw_fast, radar_param->alpha, radar_param->weight, radar_param->thrsh_db, radar_param->cal_period, radar_param->lower, radar_param->upper, radar_param->search, radar_param->window_func, radar_param->dc_cut, radar_param->output, radar_param->nfft, radar_param->period, radar_param->enlpf);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_PARAM, (const char *)radar_param, MAVLINK_MSG_ID_RADAR_PARAM_MIN_LEN, MAVLINK_MSG_ID_RADAR_PARAM_LEN, MAVLINK_MSG_ID_RADAR_PARAM_CRC);
#endif
}

#if MAVLINK_MSG_ID_RADAR_PARAM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radar_param_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t sensing_mode, uint32_t data_sel, uint32_t decimate, uint32_t rx_gain, float fout_ch1, uint32_t fmcw_t, float fmcw_f, uint8_t fmcw_fast, float alpha, uint32_t weight, float thrsh_db, uint32_t cal_period, float lower, float upper, uint32_t search, uint32_t window_func, uint32_t dc_cut, uint32_t output, uint32_t nfft, float period, uint8_t enlpf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, sensing_mode);
    _mav_put_uint32_t(buf, 4, data_sel);
    _mav_put_uint32_t(buf, 8, decimate);
    _mav_put_uint32_t(buf, 12, rx_gain);
    _mav_put_float(buf, 16, fout_ch1);
    _mav_put_uint32_t(buf, 20, fmcw_t);
    _mav_put_float(buf, 24, fmcw_f);
    _mav_put_float(buf, 28, alpha);
    _mav_put_uint32_t(buf, 32, weight);
    _mav_put_float(buf, 36, thrsh_db);
    _mav_put_uint32_t(buf, 40, cal_period);
    _mav_put_float(buf, 44, lower);
    _mav_put_float(buf, 48, upper);
    _mav_put_uint32_t(buf, 52, search);
    _mav_put_uint32_t(buf, 56, window_func);
    _mav_put_uint32_t(buf, 60, dc_cut);
    _mav_put_uint32_t(buf, 64, output);
    _mav_put_uint32_t(buf, 68, nfft);
    _mav_put_float(buf, 72, period);
    _mav_put_uint8_t(buf, 76, fmcw_fast);
    _mav_put_uint8_t(buf, 77, enlpf);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_PARAM, buf, MAVLINK_MSG_ID_RADAR_PARAM_MIN_LEN, MAVLINK_MSG_ID_RADAR_PARAM_LEN, MAVLINK_MSG_ID_RADAR_PARAM_CRC);
#else
    mavlink_radar_param_t *packet = (mavlink_radar_param_t *)msgbuf;
    packet->sensing_mode = sensing_mode;
    packet->data_sel = data_sel;
    packet->decimate = decimate;
    packet->rx_gain = rx_gain;
    packet->fout_ch1 = fout_ch1;
    packet->fmcw_t = fmcw_t;
    packet->fmcw_f = fmcw_f;
    packet->alpha = alpha;
    packet->weight = weight;
    packet->thrsh_db = thrsh_db;
    packet->cal_period = cal_period;
    packet->lower = lower;
    packet->upper = upper;
    packet->search = search;
    packet->window_func = window_func;
    packet->dc_cut = dc_cut;
    packet->output = output;
    packet->nfft = nfft;
    packet->period = period;
    packet->fmcw_fast = fmcw_fast;
    packet->enlpf = enlpf;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_PARAM, (const char *)packet, MAVLINK_MSG_ID_RADAR_PARAM_MIN_LEN, MAVLINK_MSG_ID_RADAR_PARAM_LEN, MAVLINK_MSG_ID_RADAR_PARAM_CRC);
#endif
}
#endif

#endif

// MESSAGE RADAR_PARAM UNPACKING


/**
 * @brief Get field sensing_mode from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_sensing_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field data_sel from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_data_sel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field decimate from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_decimate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field rx_gain from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_rx_gain(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field fout_ch1 from radar_param message
 *
 * @return float
 */
static inline float mavlink_msg_radar_param_get_fout_ch1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field fmcw_t from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_fmcw_t(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field fmcw_f from radar_param message
 *
 * @return float
 */
static inline float mavlink_msg_radar_param_get_fmcw_f(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field fmcw_fast from radar_param message
 *
 * @return uint8_t
 */
static inline uint8_t mavlink_msg_radar_param_get_fmcw_fast(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  76);
}

/**
 * @brief Get field alpha from radar_param message
 *
 * @return float
 */
static inline float mavlink_msg_radar_param_get_alpha(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field weight from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_weight(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Get field thrsh_db from radar_param message
 *
 * @return float
 */
static inline float mavlink_msg_radar_param_get_thrsh_db(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field cal_period from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_cal_period(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  40);
}

/**
 * @brief Get field lower from radar_param message
 *
 * @return float
 */
static inline float mavlink_msg_radar_param_get_lower(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field upper from radar_param message
 *
 * @return float
 */
static inline float mavlink_msg_radar_param_get_upper(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field search from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_search(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  52);
}

/**
 * @brief Get field window_func from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_window_func(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  56);
}

/**
 * @brief Get field dc_cut from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_dc_cut(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  60);
}

/**
 * @brief Get field output from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_output(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  64);
}

/**
 * @brief Get field nfft from radar_param message
 *
 * @return uint32_t
 */
static inline uint32_t mavlink_msg_radar_param_get_nfft(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  68);
}

/**
 * @brief Get field period from radar_param message
 *
 * @return float
 */
static inline float mavlink_msg_radar_param_get_period(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field enlpf from radar_param message
 *
 * @return uint8_t
 */
static inline uint8_t mavlink_msg_radar_param_get_enlpf(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  77);
}

/**
 * @brief Decode a radar_param message into a struct
 *
 * @param msg The message to decode
 * @param radar_param C-struct to decode the message contents into
 */
static inline void mavlink_msg_radar_param_decode(const mavlink_message_t* msg, mavlink_radar_param_t* radar_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    radar_param->sensing_mode = mavlink_msg_radar_param_get_sensing_mode(msg);
    radar_param->data_sel = mavlink_msg_radar_param_get_data_sel(msg);
    radar_param->decimate = mavlink_msg_radar_param_get_decimate(msg);
    radar_param->rx_gain = mavlink_msg_radar_param_get_rx_gain(msg);
    radar_param->fout_ch1 = mavlink_msg_radar_param_get_fout_ch1(msg);
    radar_param->fmcw_t = mavlink_msg_radar_param_get_fmcw_t(msg);
    radar_param->fmcw_f = mavlink_msg_radar_param_get_fmcw_f(msg);
    radar_param->alpha = mavlink_msg_radar_param_get_alpha(msg);
    radar_param->weight = mavlink_msg_radar_param_get_weight(msg);
    radar_param->thrsh_db = mavlink_msg_radar_param_get_thrsh_db(msg);
    radar_param->cal_period = mavlink_msg_radar_param_get_cal_period(msg);
    radar_param->lower = mavlink_msg_radar_param_get_lower(msg);
    radar_param->upper = mavlink_msg_radar_param_get_upper(msg);
    radar_param->search = mavlink_msg_radar_param_get_search(msg);
    radar_param->window_func = mavlink_msg_radar_param_get_window_func(msg);
    radar_param->dc_cut = mavlink_msg_radar_param_get_dc_cut(msg);
    radar_param->output = mavlink_msg_radar_param_get_output(msg);
    radar_param->nfft = mavlink_msg_radar_param_get_nfft(msg);
    radar_param->period = mavlink_msg_radar_param_get_period(msg);
    radar_param->fmcw_fast = mavlink_msg_radar_param_get_fmcw_fast(msg);
    radar_param->enlpf = mavlink_msg_radar_param_get_enlpf(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RADAR_PARAM_LEN? msg->len : MAVLINK_MSG_ID_RADAR_PARAM_LEN;
        memset(radar_param, 0, MAVLINK_MSG_ID_RADAR_PARAM_LEN);
    memcpy(radar_param, _MAV_PAYLOAD(msg), len);
#endif
}
