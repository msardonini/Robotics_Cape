/** @file
 *    @brief MAVLink comm protocol testsuite generated from radar.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef RADAR_TESTSUITE_H
#define RADAR_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_aurora(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_radar(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_aurora(system_id, component_id, last_msg);
    mavlink_test_radar(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"
#include "../aurora/testsuite.h"


static void mavlink_test_radar_param(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RADAR_PARAM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_radar_param_t packet_in = {
        963497464,963497672,963497880,963498088,129.0,963498504,185.0,213.0,963499128,269.0,963499544,325.0,353.0,963500168,963500376,963500584,963500792,963501000,521.0,233,44
    };
    mavlink_radar_param_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sensing_mode = packet_in.sensing_mode;
        packet1.data_sel = packet_in.data_sel;
        packet1.decimate = packet_in.decimate;
        packet1.rx_gain = packet_in.rx_gain;
        packet1.fout_ch1 = packet_in.fout_ch1;
        packet1.fmcw_t = packet_in.fmcw_t;
        packet1.fmcw_f = packet_in.fmcw_f;
        packet1.alpha = packet_in.alpha;
        packet1.weight = packet_in.weight;
        packet1.thrsh_db = packet_in.thrsh_db;
        packet1.cal_period = packet_in.cal_period;
        packet1.lower = packet_in.lower;
        packet1.upper = packet_in.upper;
        packet1.search = packet_in.search;
        packet1.window_func = packet_in.window_func;
        packet1.dc_cut = packet_in.dc_cut;
        packet1.output = packet_in.output;
        packet1.nfft = packet_in.nfft;
        packet1.period = packet_in.period;
        packet1.fmcw_fast = packet_in.fmcw_fast;
        packet1.enlpf = packet_in.enlpf;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RADAR_PARAM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RADAR_PARAM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_param_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_radar_param_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_param_pack(system_id, component_id, &msg , packet1.sensing_mode , packet1.data_sel , packet1.decimate , packet1.rx_gain , packet1.fout_ch1 , packet1.fmcw_t , packet1.fmcw_f , packet1.fmcw_fast , packet1.alpha , packet1.weight , packet1.thrsh_db , packet1.cal_period , packet1.lower , packet1.upper , packet1.search , packet1.window_func , packet1.dc_cut , packet1.output , packet1.nfft , packet1.period , packet1.enlpf );
    mavlink_msg_radar_param_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_param_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sensing_mode , packet1.data_sel , packet1.decimate , packet1.rx_gain , packet1.fout_ch1 , packet1.fmcw_t , packet1.fmcw_f , packet1.fmcw_fast , packet1.alpha , packet1.weight , packet1.thrsh_db , packet1.cal_period , packet1.lower , packet1.upper , packet1.search , packet1.window_func , packet1.dc_cut , packet1.output , packet1.nfft , packet1.period , packet1.enlpf );
    mavlink_msg_radar_param_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_radar_param_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_param_send(MAVLINK_COMM_1 , packet1.sensing_mode , packet1.data_sel , packet1.decimate , packet1.rx_gain , packet1.fout_ch1 , packet1.fmcw_t , packet1.fmcw_f , packet1.fmcw_fast , packet1.alpha , packet1.weight , packet1.thrsh_db , packet1.cal_period , packet1.lower , packet1.upper , packet1.search , packet1.window_func , packet1.dc_cut , packet1.output , packet1.nfft , packet1.period , packet1.enlpf );
    mavlink_msg_radar_param_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_radar_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RADAR_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_radar_command_t packet_in = {
        963497464,963497672,29
    };
    mavlink_radar_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.address = packet_in.address;
        packet1.data = packet_in.data;
        packet1.command = packet_in.command;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RADAR_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RADAR_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_radar_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_command_pack(system_id, component_id, &msg , packet1.command , packet1.address , packet1.data );
    mavlink_msg_radar_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command , packet1.address , packet1.data );
    mavlink_msg_radar_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_radar_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_command_send(MAVLINK_COMM_1 , packet1.command , packet1.address , packet1.data );
    mavlink_msg_radar_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_radar(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_radar_param(system_id, component_id, last_msg);
    mavlink_test_radar_command(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // RADAR_TESTSUITE_H
