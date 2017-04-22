/** @file
 *    @brief MAVLink comm protocol testsuite generated from aurora.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef AURORA_TESTSUITE_H
#define AURORA_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_aurora(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_aurora(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_radar(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RADAR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_radar_t packet_in = {
        963497464,{ 17443, 17444, 17445, 17446, 17447 },{ 175, 176, 177, 178, 179 },{ 254, 255, 0, 1, 2 }
    };
    mavlink_radar_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.n = packet_in.n;
        
        mav_array_memcpy(packet1.distance, packet_in.distance, sizeof(uint16_t)*5);
        mav_array_memcpy(packet1.up_mag, packet_in.up_mag, sizeof(uint8_t)*5);
        mav_array_memcpy(packet1.down_mag, packet_in.down_mag, sizeof(uint8_t)*5);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RADAR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RADAR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_radar_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_pack(system_id, component_id, &msg , packet1.n , packet1.distance , packet1.up_mag , packet1.down_mag );
    mavlink_msg_radar_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.n , packet1.distance , packet1.up_mag , packet1.down_mag );
    mavlink_msg_radar_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_radar_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_send(MAVLINK_COMM_1 , packet1.n , packet1.distance , packet1.up_mag , packet1.down_mag );
    mavlink_msg_radar_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_radar_filtered(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RADAR_FILTERED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_radar_filtered_t packet_in = {
        { 17.0, 18.0, 19.0, 20.0, 21.0 },{ 157.0, 158.0, 159.0, 160.0, 161.0 },{ 297.0, 298.0, 299.0, 300.0, 301.0 }
    };
    mavlink_radar_filtered_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.distance, packet_in.distance, sizeof(float)*5);
        mav_array_memcpy(packet1.confidence, packet_in.confidence, sizeof(float)*5);
        mav_array_memcpy(packet1.velocity, packet_in.velocity, sizeof(float)*5);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RADAR_FILTERED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RADAR_FILTERED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_filtered_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_radar_filtered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_filtered_pack(system_id, component_id, &msg , packet1.distance , packet1.confidence , packet1.velocity );
    mavlink_msg_radar_filtered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_filtered_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.distance , packet1.confidence , packet1.velocity );
    mavlink_msg_radar_filtered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_radar_filtered_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radar_filtered_send(MAVLINK_COMM_1 , packet1.distance , packet1.confidence , packet1.velocity );
    mavlink_msg_radar_filtered_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_aurora(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_radar(system_id, component_id, last_msg);
    mavlink_test_radar_filtered(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // AURORA_TESTSUITE_H
