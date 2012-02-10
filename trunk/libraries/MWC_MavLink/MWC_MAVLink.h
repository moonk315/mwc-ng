#ifndef MWC_MAVLink_h
#define MWC_MAVLink_h

#define MAVLINK_SEPARATE_HELPERS
#define MAVLINK_EXTERNAL_RX_BUFFER 1 
#define m_mavlink_message recv_msg

#include "include/mwc_ng/version.h"
//#include "include/mwc_ng/mwc_ng.h"

// this allows us to make mavlink_message_t much smaller
#define MAVLINK_MAX_PAYLOAD_LEN MAVLINK_MAX_DIALECT_PAYLOAD_SIZE

#define MAVLINK_COMM_NUM_BUFFERS 1
#include "include/mavlink_types.h"


/// MAVLink system definition
extern mavlink_system_t mavlink_system;
extern void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
extern mavlink_message_t recv_msg[MAVLINK_COMM_NUM_BUFFERS];
//extern uint8_t comm_receive_ch(mavlink_channel_t chan);
//extern uint16_t comm_get_available(mavlink_channel_t chan);
//extern int comm_get_txspace(mavlink_channel_t chan);


#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "include/mwc_ng/mavlink.h"


#include "include/mavlink_helpers.h"


#endif 
