/**
 * MultiWii NG 0.1 - 2012
 * Process Output. (pid) -> (out) -> <core>
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
 
#if (FRAME == GIMBAL) || (FRAME == FLYING_WING)
  #define NUMBER_MOTOR 0
  #define FRAME_SERVOS 9
#elif (FRAME == BI)
  #define NUMBER_MOTOR 2
  #define FRAME_SERVOS 2
#elif (FRAME == TRI)
  #define NUMBER_MOTOR 3
  #define FRAME_SERVOS 1
#elif (FRAME == QUADP) || (FRAME == QUADX) || (FRAME == Y4)
  #define NUMBER_MOTOR 4
  #define FRAME_SERVOS 0
#elif (FRAME == Y6) || (FRAME == HEX6) || (FRAME == HEX6X)
  #define NUMBER_MOTOR 6
  #define FRAME_SERVOS 0
#elif (FRAME == OCTOX8) || (FRAME == OCTOFLATP) || (FRAME == OCTOFLATX)
  #define NUMBER_MOTOR 8
  #define FRAME_SERVOS 0
#endif

#if defined(CAM_MOUNT)
  #define CAM_SERVOS 2
#else  
  #define CAM_SERVOS 0
#endif

#define PIDMIX(X,Y,Z) (PWM_ESC_IDLE_THROTTLE + pid.ctrl.throttle + pid.ctrl.roll*X + pid.ctrl.pitch*Y + YAW_DIRECTION * pid.ctrl.yaw*Z)

void process_frame_mixes() {
  int16_t maxMotor, minMotor,a;
  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    // !!!! WTF !!!!
    pid.ctrl.yaw = constrain(pid.ctrl.yaw, -100 - abs(input.ctrl.yaw), +100 + abs(input.ctrl.yaw));
  #endif
  #if (FRAME == BI)
    out.motor[0] = PIDMIX(+1, 0, 0); //LEFT
    out.motor[1] = PIDMIX(-1, 0, 0); //RIGHT        
    out.servo[0]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000); //LEFT
    out.servo[1]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000); //RIGHT
  #endif
  #if (FRAME == TRI)
    out.motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    out.motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    out.motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    out.servo[0] = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
  #endif
  #if (FRAME == QUADP)
    out.motor[0] = PIDMIX( 0,+1,-1); //REAR
    out.motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    out.motor[2] = PIDMIX(+1, 0,+1); //LEFT
    out.motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #endif
  #if (FRAME == QUADX)
    out.motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    out.motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    out.motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    out.motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #endif
  #if (FRAME == Y4)
    out.motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
    out.motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
    out.motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
    out.motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
  #endif
  #if (FRAME == Y6)
    out.motor[0] = PIDMIX(+0,+4/3,+1); //REAR
    out.motor[1] = PIDMIX(-1,-2/3,-1); //RIGHT
    out.motor[2] = PIDMIX(+1,-2/3,-1); //LEFT
    out.motor[3] = PIDMIX(+0,+4/3,-1); //UNDER_REAR
    out.motor[4] = PIDMIX(-1,-2/3,+1); //UNDER_RIGHT
    out.motor[5] = PIDMIX(+1,-2/3,+1); //UNDER_LEFT    
  #endif
  #if (FRAME == HEX6)
    out.motor[0] = PIDMIX(-1/2,+1/2,+1); //REAR_R
    out.motor[1] = PIDMIX(-1/2,-1/2,-1); //FRONT_R
    out.motor[2] = PIDMIX(+1/2,+1/2,+1); //REAR_L
    out.motor[3] = PIDMIX(+1/2,-1/2,-1); //FRONT_L
    out.motor[4] = PIDMIX(+0  ,-1  ,+1); //FRONT
    out.motor[5] = PIDMIX(+0  ,+1  ,-1); //REAR
  #endif
  #if (FRAME == HEX6X)
    out.motor[0] = PIDMIX(-1/2,+1/2,+1); //REAR_R
    out.motor[1] = PIDMIX(-1/2,-1/2,+1); //FRONT_R
    out.motor[2] = PIDMIX(+1/2,+1/2,-1); //REAR_L
    out.motor[3] = PIDMIX(+1/2,-1/2,-1); //FRONT_L
    out.motor[4] = PIDMIX(-1  ,+0  ,-1); //RIGHT
    out.motor[5] = PIDMIX(+1  ,+0  ,+1); //LEFT
  #endif
  #if (FRAME == OCTOX8)
    out.motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    out.motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    out.motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    out.motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
    out.motor[4] = PIDMIX(-1,+1,+1); //UNDER_REAR_R
    out.motor[5] = PIDMIX(-1,-1,-1); //UNDER_FRONT_R
    out.motor[6] = PIDMIX(+1,+1,-1); //UNDER_REAR_L
    out.motor[7] = PIDMIX(+1,-1,+1); //UNDER_FRONT_L
  #endif
  #if (FRAME == OCTOFLATP)
    out.motor[0] = PIDMIX(+7/10,-7/10,+1); //FRONT_L
    out.motor[1] = PIDMIX(-7/10,-7/10,+1); //FRONT_R
    out.motor[2] = PIDMIX(-7/10,+7/10,+1); //REAR_R
    out.motor[3] = PIDMIX(+7/10,+7/10,+1); //REAR_L
    out.motor[4] = PIDMIX(+0   ,-1   ,-1); //FRONT
    out.motor[5] = PIDMIX(-1   ,+0   ,-1); //RIGHT
    out.motor[6] = PIDMIX(+0   ,+1   ,-1); //REAR
    out.motor[7] = PIDMIX(-1   ,+0   ,-1); //LEFT 
  #endif
  #if (FRAME == OCTOFLATX)
    out.motor[0] = PIDMIX(+1  ,-1/2,+1); //MIDFRONT_L
    out.motor[1] = PIDMIX(-1/2,-1  ,+1); //FRONT_R
    out.motor[2] = PIDMIX(-1  ,+1/2,+1); //MIDREAR_R
    out.motor[3] = PIDMIX(+1/2,+1  ,+1); //REAR_L
    out.motor[4] = PIDMIX(+1/2,-1  ,-1); //FRONT_L
    out.motor[5] = PIDMIX(-1  ,-1/2,-1); //MIDFRONT_R
    out.motor[6] = PIDMIX(-1/2,+1  ,-1); //REAR_R
    out.motor[7] = PIDMIX(+1  ,+1/2,-1); //MIDREAR_L 
  #endif
  #if (FRAME == GIMBAL)
    out.servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] /16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
    out.servo[2] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP   * angle[ROLL]  /16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
  #endif
  #if (FRAME == FLYING_WING)
    out.servo[1]  = constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
    out.servo[2]  = constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //RIGHT
  #endif
  maxMotor = minMotor = out.motor[0];
  for(uint8_t i = 1; i < NUMBER_MOTOR; i++) {
    if (out.motor[i] > maxMotor) maxMotor = out.motor[i];
    if (out.motor[i] < minMotor) minMotor = out.motor[i];
  }  
  for (uint8_t i = 0; i < NUMBER_MOTOR; i++) {
    if (maxMotor > PWM_ESC_MAX_THROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
      out.motor[i] -= maxMotor - PWM_ESC_MAX_THROTTLE;
    if (minMotor < PWM_ESC_IDLE_THROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its min.
      out.motor[i] += PWM_ESC_IDLE_THROTTLE - minMotor;
    if (out.motors_armed)  
      out.motor[i] = constrain(out.motor[i], PWM_ESC_IDLE_THROTTLE, PWM_ESC_MAX_THROTTLE);    
    else  
      out.motor[i] = PWM_ESC_MIN_THROTTLE; 
    PWMOut(i, out.motor[i]);
  }
}

inline void Output_Init() {
  out.motor_cnt = NUMBER_MOTOR;
  out.servo_cnt = FRAME_SERVOS + CAM_SERVOS;
}  

inline void Output_loop_100hz() {
  process_frame_mixes();
}  


 
