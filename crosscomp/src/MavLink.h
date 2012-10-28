/**
 * MultiWii NG 0.1 - 2012
 * MAVLink protocol implementation
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <MWC_MAVLink.h>

#define MAV_HEARTBEAT_TIME       (500000L)
#define MAV_PARAM_TIME           (50000L / 2)

mavlink_system_t mavlink_system;
mavlink_message_t recv_msg[MAVLINK_COMM_NUM_BUFFERS];
const mavlink_param_set_t *mavlink_param_set = (mavlink_param_set_t *) _MAV_PAYLOAD(&recv_msg[0]);
const mavlink_param_request_read_t *mavlink_param_request_read = (mavlink_param_request_read_t *) _MAV_PAYLOAD(&recv_msg[0]);
const mavlink_request_data_stream_t *mavlink_request_data_stream = (mavlink_request_data_stream_t *) _MAV_PAYLOAD(&recv_msg[0]);
const mavlink_command_long_t *mavlink_command_long = (mavlink_command_long_t *) _MAV_PAYLOAD(&recv_msg[0]);

typedef struct mav_context_data mav_context_data_t;
struct mav_context_data {
  mavlink_message_t snd_msg;
  mavlink_status_t  recv_status;
  struct pt_sem mavlink_channel_mutex;
  struct pt_sem mavlink_system_mutex;
  struct pt thread_mav_link_scheduler_heartbeat_pt;
  struct pt thread_mav_link_scheduler_raw_sensors_stream_pt;
  struct pt thread_mav_link_scheduler_ext1_stream_pt;
  struct pt thread_mav_link_scheduler_rc_channels_stream_pt;
  struct pt thread_mav_link_receiver_pt;
  uint8_t req_param;
  uint8_t param_idx;
  uint8_t raw_sensors_stream_enabled:1;
  uint8_t rc_channels_stream_enabled:1;
  uint8_t ext1_stream_enabled:1;
  param_search_rec_t sr;
  uint16_t last_command;
  uint8_t  cmd_result;
  uint16_t packet_drops;
};
mav_context_data_t mav;

static struct timer_big timer_heartbeat           = {0, MAV_HEARTBEAT_TIME * 2};
static struct timer_big timer_raw_sensors_stream  = {0, 20000};
static struct timer_big timer_rc_channels_stream  = {0, 20000};
static struct timer_big timer_ext1_stream         = {0, 20000};

inline uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid) {
    return (sysid = mavlink_system.sysid);
}

inline void pack_heartbeat() {
  uint8_t system_state = MAV_STATE_UNINIT;
  uint8_t system_mode = MAV_MODE_MANUAL_DISARMED;
  switch (flight.sys_state) {
    case SYS_STATE_IDLE:
    case SYS_STATE_ARM_REQ:
      system_state = MAV_STATE_STANDBY;
      system_mode = MAV_MODE_MANUAL_DISARMED;
      break;
    case SYS_STATE_CALIBRATING:
      system_state = MAV_STATE_CALIBRATING;
      system_mode = MAV_MODE_MANUAL_DISARMED;
      break;
    case SYS_STATE_ARMED:
    case SYS_STATE_DISARM_REQ:
    case SYS_STATE_FLIGHT:
      system_state = MAV_STATE_ACTIVE;
      system_mode = MAV_MODE_MANUAL_ARMED;
      break;
  }
  mavlink_msg_heartbeat_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg,
    mavlink_system.type,
    MAV_AUTOPILOT_GENERIC,
    system_mode,
    flight.sys_state,
    system_state);
}

inline void pack_sys_status() {
  uint32_t sensors = 0;
  if (ACC  != _NONE_) sensors |=_BV(1);
  if (MAG  != _NONE_) sensors |=_BV(2);
  if (BARO != _NONE_) sensors |=_BV(3);
  mavlink_msg_sys_status_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg,
    sensors,
    sensors,
    sensors,
    (((uint32_t)cpu_util_pct * 1000) >> 8),
    batt_voltage,
    -1, /*No current sensor*/
    -1, /*No battery estimation*/
    mav.packet_drops,
    twi_err_cnt,
    0,
    0,
    0,
    0);
}

inline void pack_attitude() {
  mavlink_msg_attitude_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg, current_time_ms,
    ahrs.eul_ref.roll,
    -ahrs.eul_ref.pitch,
    -ahrs.eul_ref.yaw,
    ahrs_get_roll_speed(),
    ahrs_get_pitch_speed(),
    ahrs_get_yaw_speed());
}

inline void pack_raw_imu() {
  mavlink_msg_raw_imu_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg, current_time_us,
    imu.acc.fr.x,
    imu.acc.fr.y,
    imu.acc.fr.z,
    imu.gyro.eul.roll,
    imu.gyro.eul.pitch,
    imu.gyro.eul.yaw,
    imu.mag.fr.x,
    imu.mag.fr.y,
    imu.mag.fr.z);
}

inline void pack_param_value() {
  // map types
  uint8_t param_type  = MAVLINK_TYPE_FLOAT;
  /*
  if (!param_is_float(&mav.sr)) {
    switch (param_get_type_kind(&mav.sr)) {
      case PARAM_TYPE_KIND_U8:
      case PARAM_TYPE_KIND_U16:
      case PARAM_TYPE_KIND_U32: param_type = MAVLINK_TYPE_UINT32_T; break;
      case PARAM_TYPE_KIND_I16:
      case PARAM_TYPE_KIND_I32: param_type = MAVLINK_TYPE_INT32_T; break;
    }
  }
  */
  mavlink_msg_param_value_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg,
    mav.sr.name,
    param_get_val(&mav.sr),
    param_type,
    sys_param_values_cnt,
    mav.param_idx);
}

inline void pack_rc_channels_raw() {
  mavlink_msg_rc_channels_raw_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg,
    current_time_ms,
    (RX / 100),
    rx_data.raw[0],
    rx_data.raw[1],
    rx_data.raw[2],
    rx_data.raw[3],
    rx_data.raw[4],
    rx_data.raw[5],
    rx_data.raw[6],
    rx_data.raw[7],
    255);
}

inline void pack_rc_channels_scaled() {
  mavlink_msg_rc_channels_scaled_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg,
    current_time_ms,
    (RX / 100),
    input.ctrl.raw[0] * 10,
    input.ctrl.raw[1] * 20,
    input.ctrl.raw[2] * 20,
    input.ctrl.raw[3] * 20,
    0,
    0,
    0,
    0,
    255);
}

inline void pack_servo_output_raw() {
  mavlink_msg_servo_output_raw_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg,
    current_time_ms,
    (FRAME / 100),
    out.motor[0],
    out.motor[1],
    out.motor[2],
    out.motor[3],
    out.motor[4],
    out.motor[5],
    out.motor[6],
    out.motor[7]);
}

inline void pack_msg_command_ack() {
  mavlink_msg_command_ack_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg,
    mav.last_command,
    mav.cmd_result);
}

inline void mav_link_process_command() {
  mav.last_command = mavlink_command_long->command;
  mav.cmd_result = MAV_RESULT_ACCEPTED;
  switch (mav.last_command) {
    case MAV_CMD_PREFLIGHT_CALIBRATION:
      if (mavlink_command_long->param1 == 1.0f) imu_calibrate_gyro();
      if (mavlink_command_long->param2 == 1.0f) imu_calibrate_acc();
      if (mavlink_command_long->param3 == 1.0f) imu_calibrate_mag_gain();
      break;
    case MAV_CMD_PREFLIGHT_STORAGE:
      if (mavlink_command_long->param1 == 0.0f) read_storage();
      if (mavlink_command_long->param1 == 1.0f) write_storage();
      break;
    default:
      mav.cmd_result = MAV_RESULT_UNSUPPORTED;
      break;
  }
};

static PT_THREAD(thread_mav_link_transmit_message(struct pt *pt, uint8_t msgid)) {
  PT_BEGIN(pt);
  PT_SEM_WAIT(pt, &mav.mavlink_channel_mutex);
  switch(msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: pack_heartbeat(); break;
    case MAVLINK_MSG_ID_SYS_STATUS: pack_sys_status(); break;
    case MAVLINK_MSG_ID_ATTITUDE: pack_attitude(); break;
    case MAVLINK_MSG_ID_RAW_IMU: pack_raw_imu(); break;
    case MAVLINK_MSG_ID_PARAM_VALUE: pack_param_value(); break;
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW: pack_rc_channels_raw(); break;
    case MAVLINK_MSG_ID_RC_CHANNELS_SCALED: pack_rc_channels_scaled(); break;
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: pack_servo_output_raw(); break;
    case MAVLINK_MSG_ID_COMMAND_ACK: pack_msg_command_ack(); break;
   // default: PT_SEM_SIGNAL(pt, &mav.mavlink_channel_mutex); PT_EXIT(pt);
  }
  static uint8_t i;
  for (i = 0; i < MAVLINK_NUM_NON_PAYLOAD_BYTES + mav.snd_msg.len; i++) {
    PT_WAIT_UNTIL(pt, !GUI_serial_tx_full());
    uint8_t *buff = (uint8_t *)&mav.snd_msg.magic;
    GUI_serial_write(buff[i]);
  };
  PT_SEM_SIGNAL(pt, &mav.mavlink_channel_mutex);
  PT_END(pt);
}

static PT_THREAD(thread_mav_link_receiver(struct pt *pt)) {
  static struct pt thread_reply_pt;
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, GUI_serial_available() && mavlink_parse_char(MAVLINK_COMM_0, GUI_serial_read(), &recv_msg[0], &mav.recv_status));
  mav.packet_drops += mav.recv_status.packet_rx_drop_count;
  if (recv_msg[0].msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    PT_SEM_INIT(&mav.mavlink_system_mutex, 50);
  }
  if (mavlink_check_target(recv_msg[0].sysid, recv_msg[0].compid)) {
    if (recv_msg[0].msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
      mav.req_param = 0;
      param_search_first(&mav.sr);
      timer_heartbeat.interval = MAV_PARAM_TIME * 2;
    } else
    if ((recv_msg[0].msgid == MAVLINK_MSG_ID_PARAM_SET) && (mav.req_param >= sys_param_values_cnt)) {
      if (param_search_by_name(&mav.sr, (char*) mavlink_param_set->param_id)) {
        param_set_val(&mav.sr, mavlink_param_set->param_value);
        PT_SPAWN(pt, &thread_reply_pt, thread_mav_link_transmit_message(&thread_reply_pt, MAVLINK_MSG_ID_PARAM_VALUE));
      }
    } else
    if ((recv_msg[0].msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) && (mav.req_param >= sys_param_values_cnt)) {
      if (mavlink_param_request_read->param_index >= 0) {
        if (param_search_by_idx(&mav.sr, mavlink_param_request_read->param_index)) {
          mav.param_idx = mavlink_param_request_read->param_index;
          PT_SPAWN(pt, &thread_reply_pt, thread_mav_link_transmit_message(&thread_reply_pt, MAVLINK_MSG_ID_PARAM_VALUE));
        }
      } else
      if (param_search_by_name(&mav.sr, (char*) mavlink_param_request_read->param_id))
        PT_SPAWN(pt, &thread_reply_pt, thread_mav_link_transmit_message(&thread_reply_pt, MAVLINK_MSG_ID_PARAM_VALUE));
    } else
    if (recv_msg[0].msgid == MAVLINK_MSG_ID_REQUEST_DATA_STREAM) {
      uint32_t inter = (2 * 1000000L) / mavlink_request_data_stream->req_message_rate;
      if ((mavlink_request_data_stream->req_stream_id == MAV_DATA_STREAM_ALL) || (mavlink_request_data_stream->req_stream_id == MAV_DATA_STREAM_RAW_SENSORS)) {
        mav.raw_sensors_stream_enabled = mavlink_request_data_stream->start_stop;
        timer_raw_sensors_stream.interval = inter;
      }
      if ((mavlink_request_data_stream->req_stream_id == MAV_DATA_STREAM_ALL) || (mavlink_request_data_stream->req_stream_id == MAV_DATA_STREAM_EXTRA1)) {
        mav.ext1_stream_enabled = mavlink_request_data_stream->start_stop;
        timer_ext1_stream.interval = inter;
      }
      if ((mavlink_request_data_stream->req_stream_id == MAV_DATA_STREAM_ALL) || (mavlink_request_data_stream->req_stream_id == MAV_DATA_STREAM_RC_CHANNELS)) {
        mav.rc_channels_stream_enabled = mavlink_request_data_stream->start_stop;
        timer_rc_channels_stream.interval = inter;
      }
    } else
    if (recv_msg[0].msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
      mav_link_process_command();
      //PT_SPAWN(pt, &thread_reply_pt, thread_mav_link_transmit_message(&thread_reply_pt, MAVLINK_MSG_ID_COMMAND_ACK));
    }
  }
  PT_END(pt);
}

static PT_THREAD(thread_mav_link_scheduler_heartbeat(struct pt *pt, uint16_t dt)) {
  static struct pt thread_send_pt;
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_heartbeat, dt));
  if (mav.req_param < sys_param_values_cnt) {
    mav.param_idx = mav.req_param;
    PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_PARAM_VALUE));
    mav.req_param++;
    if (!param_search_next(&mav.sr))
      timer_heartbeat.interval = MAV_HEARTBEAT_TIME * 2;
  } else {
    PT_SEM_WAIT(pt, &mav.mavlink_system_mutex);
    PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_HEARTBEAT));
    PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_SYS_STATUS));
  }
  PT_END(pt);
}

static PT_THREAD(thread_mav_link_scheduler_raw_sensors_stream(struct pt *pt, uint16_t dt)) {
  static struct pt thread_send_pt;
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, mav.raw_sensors_stream_enabled && timer_expired(&timer_raw_sensors_stream, dt));
  PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_RAW_IMU));
  PT_END(pt);
}

static PT_THREAD(thread_mav_link_scheduler_rc_channels_stream(struct pt *pt, uint16_t dt)) {
  static struct pt thread_send_pt;
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, mav.rc_channels_stream_enabled && timer_expired(&timer_rc_channels_stream, dt));
  PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_RC_CHANNELS_RAW));
  PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_RC_CHANNELS_SCALED));
  PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW));
  PT_END(pt);
}

static PT_THREAD(thread_mav_link_scheduler_ext1_stream(struct pt *pt, uint16_t dt)) {
  static struct pt thread_send_pt;
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, mav.ext1_stream_enabled && timer_expired(&timer_ext1_stream, dt));
  PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_ATTITUDE));
  PT_END(pt);
}

inline void MavLink_Init() {
  mavlink_system.sysid = 1;
  mavlink_system.compid = MAV_COMP_ID_IMU;
  mavlink_system.type = MAV_TYPE_QUADROTOR;
  mav.req_param = 255;
  PT_INIT(&mav.thread_mav_link_scheduler_heartbeat_pt);
  PT_INIT(&mav.thread_mav_link_scheduler_raw_sensors_stream_pt);
  PT_INIT(&mav.thread_mav_link_scheduler_ext1_stream_pt);
  PT_INIT(&mav.thread_mav_link_scheduler_rc_channels_stream_pt);
  PT_INIT(&mav.thread_mav_link_receiver_pt);
  PT_SEM_INIT(&mav.mavlink_channel_mutex, 1);
}

inline void MavLink_Loop(uint16_t currentTime) {
  PT_SCHEDULE(thread_mav_link_receiver(&mav.thread_mav_link_receiver_pt));
  if (mav.mavlink_system_mutex.count > 0) {
    PT_SCHEDULE(thread_mav_link_scheduler_heartbeat(&mav.thread_mav_link_scheduler_heartbeat_pt, currentTime));
    PT_SCHEDULE(thread_mav_link_scheduler_raw_sensors_stream(&mav.thread_mav_link_scheduler_raw_sensors_stream_pt, currentTime));
    PT_SCHEDULE(thread_mav_link_scheduler_ext1_stream(&mav.thread_mav_link_scheduler_ext1_stream_pt, currentTime));
    PT_SCHEDULE(thread_mav_link_scheduler_rc_channels_stream(&mav.thread_mav_link_scheduler_rc_channels_stream_pt, currentTime));
  }
}




