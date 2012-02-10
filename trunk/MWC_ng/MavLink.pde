#include <MWC_MAVLink.h>


mavlink_system_t mavlink_system;
mavlink_message_t recv_msg[MAVLINK_COMM_NUM_BUFFERS]; 


typedef struct mav_context_data mav_context_data_t;
struct mav_context_data {
  mavlink_message_t snd_msg;
  mavlink_status_t  recv_status;
  struct pt_sem mavlink_channel_mutex;
  struct pt_sem mavlink_system_mutex;
  struct pt thread_mav_link_scheduler_pt;
  struct pt thread_mav_link_receiver_pt;
};
mav_context_data_t mav;

static struct timer_big timer_heartbeat  = {0, SERVICE_LOOP_TIME * 2 / 5};  

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
  uint32_t sensors = 1;
  if (ACC  != _NONE_) sensors !=_BV(1);
  if (MAG  != _NONE_) sensors !=_BV(2);
  if (BARO != _NONE_) sensors !=_BV(3);
  mavlink_msg_sys_status_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg, 
    sensors, 
    sensors, 
    sensors, 
    (((uint32_t)cpu_util_pct * 1000) >> 8), 
    0, 
    0, 
    0, 
    0, 
    0, 
    0, 
    0, 
    0, 
    0); 
}  

inline void pack_attitude() {
  mavlink_msg_attitude_pack_inline(mavlink_system.sysid,  mavlink_system.compid, &mav.snd_msg, current_time_ms, 
    radians(ahrs.eul_ref.roll / 100.0), 
    radians(-ahrs.eul_ref.pitch / 100.0), 
    radians(ahrs.eul_ref.yaw / 100.0), 
    0, 
    0, 
    0);
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

static PT_THREAD(thread_mav_link_transmit_message(struct pt *pt, uint8_t msgid)) {
  static uint8_t len;
  PT_BEGIN(pt);
  PT_SEM_WAIT(pt, &mav.mavlink_channel_mutex);
  switch(msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: pack_heartbeat(); break;
    case MAVLINK_MSG_ID_SYS_STATUS: pack_sys_status(); break;
    case MAVLINK_MSG_ID_ATTITUDE: pack_attitude(); break;
    case MAVLINK_MSG_ID_RAW_IMU: pack_raw_imu(); break;
    default: PT_SEM_SIGNAL(pt, &mav.mavlink_channel_mutex); PT_EXIT(pt);
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
  if (recv_msg[0].msgid == MAVLINK_MSG_ID_HEARTBEAT)
    PT_SEM_INIT(&mav.mavlink_system_mutex, 25 * 5); 
  PT_END(pt);
}  

static PT_THREAD(thread_mav_link_scheduler(struct pt *pt, uint16_t dt)) {
  static struct pt thread_send_pt;
  PT_BEGIN(pt);
  PT_SEM_WAIT(pt, &mav.mavlink_system_mutex);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_heartbeat, dt));
  DebugLEDToggle();
  PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_HEARTBEAT));
  PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_SYS_STATUS));
  PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_ATTITUDE));
  PT_SPAWN(pt, &thread_send_pt, thread_mav_link_transmit_message(&thread_send_pt, MAVLINK_MSG_ID_RAW_IMU));
  PT_END(pt);
}

inline void MavLink_Init() {
  mavlink_system.sysid = 1;
  mavlink_system.compid = MAV_COMP_ID_IMU;
  mavlink_system.type = MAV_TYPE_QUADROTOR;
  PT_INIT(&mav.thread_mav_link_scheduler_pt);
  PT_INIT(&mav.thread_mav_link_receiver_pt);
  PT_SEM_INIT(&mav.mavlink_channel_mutex, 1); 
}  

inline void MavLink_Loop(uint16_t currentTime) {
  PT_SCHEDULE(thread_mav_link_scheduler(&mav.thread_mav_link_scheduler_pt, currentTime)); 
  PT_SCHEDULE(thread_mav_link_receiver(&mav.thread_mav_link_receiver_pt)); 
}  




