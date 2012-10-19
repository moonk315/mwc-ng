unit UAHRS;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, UTypes, Math;

var
  imu : TIMU;
  ahrs: TAHRS;
  q: TFpQ;

const
   GYRO_LSB = 14.375;
   OUTER_CTRL_LOOP_TIME = 4000 * 3;
   GYRO_SCALE = ((PI)/(GYRO_LSB * 180.0 * 1000000.0) * OUTER_CTRL_LOOP_TIME / 256.0);
   ACC_LPF_FACTOR = 64;
   INV_LPF_CONST = (1.0/ACC_LPF_FACTOR);
   GYR_CMPF_FACTOR = 270.0;
   INV_GYR_CMPF_FACTOR  =  (1.0 / (GYR_CMPF_FACTOR  + 1.0));

   _NONE_ = 0;
   MAG = 1;


procedure rotate_vectors();
procedure AHRS_Init();
procedure AHRS_loop_outer();
procedure AHRS_loop_acc();
procedure rotate_acc_vector();


implementation

function ssin(x: float): float;
begin
  Result := x;
end;

function scos(x: float): float;
begin
  Result := 1.0;
end;


procedure integrate_rotation();
var
  norm: float;
begin
  q.v.x += (-q.v.y*imu.gyro_ahrs.eul.yaw + q.v.z*imu.gyro_ahrs.eul.pitch + q.w*imu.gyro_ahrs.eul.roll)*(GYRO_SCALE*0.5);
  q.v.y += (q.v.x*imu.gyro_ahrs.eul.yaw  - q.v.z*imu.gyro_ahrs.eul.roll   + q.w*imu.gyro_ahrs.eul.pitch)*(GYRO_SCALE*0.5);
  q.v.z += (-q.v.x*imu.gyro_ahrs.eul.pitch + q.v.y*imu.gyro_ahrs.eul.roll   + q.w*imu.gyro_ahrs.eul.yaw)*(GYRO_SCALE*0.5);
  q.w   += (-q.v.x*imu.gyro_ahrs.eul.roll   - q.v.y*imu.gyro_ahrs.eul.pitch - q.v.z*imu.gyro_ahrs.eul.yaw)*(GYRO_SCALE*0.5);

  norm := 1.0/sqrt(q.v.x*q.v.x + q.v.y*q.v.y + q.v.z*q.v.z + q.w*q.w);
  q.v.x *= norm;
  q.v.y *= norm;
  q.v.z *= norm;
  q.w   *= norm;
end;

procedure integrate_euler();
begin
  //ahrs.eul_ref3_yaw += imu.gyro_ahrs.eul.yaw * GYRO_SCALE;
  //ahrs.eul_ref3_roll += imu.gyro_ahrs.eul.roll * GYRO_SCALE;
  //ahrs.eul_ref3_pitch += imu.gyro_ahrs.eul.pitch * GYRO_SCALE;
end;

procedure extract_euler();
begin
  ahrs.eul_ref2_yaw := arctan2(2*q.v.z*q.w - 2*q.v.x*q.v.y , 1 - 2*q.v.z*q.v.z - 2*q.v.y*q.v.y);
  ahrs.eul_ref2_pitch := arcsin(2*q.v.x*q.v.z + 2*q.v.y*q.w);
  ahrs.eul_ref2_roll := arctan2(2*q.v.x*q.w - 2*q.v.z*q.v.y , 1 - 2*q.v.x*q.v.x - 2*q.v.y*q.v.y);
end;

procedure rotate_vectors();
var
  deltaGyroAngle: float;
  inv_mag: float;
  ax, ay, cs, si: float;
begin
  // Rotate Estimated vector(s), YAW
  deltaGyroAngle  := -(imu.gyro_ahrs.eul.yaw - ahrs.acc_err.z) * GYRO_SCALE;
  ahrs.est_grav.x :=  scos(deltaGyroAngle) * ahrs.est_grav.x - ssin(deltaGyroAngle) * ahrs.est_grav.y;
  ahrs.est_grav.y :=  ssin(deltaGyroAngle) * ahrs.est_grav.x + scos(deltaGyroAngle) * ahrs.est_grav.y;
  if (MAG <> _NONE_) then
  begin
    ahrs.est_mag.x :=  scos(deltaGyroAngle) * ahrs.est_mag.x - ssin(deltaGyroAngle) * ahrs.est_mag.y;
    ahrs.est_mag.y :=  ssin(deltaGyroAngle) * ahrs.est_mag.x + scos(deltaGyroAngle) * ahrs.est_mag.y;
  end;
  // Rotate Estimated vector(s), ROLL
   deltaGyroAngle  := -(imu.gyro_ahrs.eul.roll - ahrs.acc_err.x) * GYRO_SCALE;
   ahrs.est_grav.y :=  scos(deltaGyroAngle) * ahrs.est_grav.y - ssin(deltaGyroAngle) * ahrs.est_grav.z;
   ahrs.est_grav.z :=  ssin(deltaGyroAngle) * ahrs.est_grav.y + scos(deltaGyroAngle) * ahrs.est_grav.z;
   if (MAG <> _NONE_) then
   begin
     ahrs.est_mag.y :=  scos(deltaGyroAngle) * ahrs.est_mag.y - ssin(deltaGyroAngle) * ahrs.est_mag.z;
     ahrs.est_mag.z :=  ssin(deltaGyroAngle) * ahrs.est_mag.y + scos(deltaGyroAngle) * ahrs.est_mag.z;
   end;
   // Rotate Estimated vector(s), PITCH
   deltaGyroAngle  := (imu.gyro_ahrs.eul.pitch - ahrs.acc_err.y) * GYRO_SCALE;
   ahrs.est_grav.z :=  scos(deltaGyroAngle) * ahrs.est_grav.z - ssin(deltaGyroAngle) * ahrs.est_grav.x;
   ahrs.est_grav.x :=  ssin(deltaGyroAngle) * ahrs.est_grav.z + scos(deltaGyroAngle) * ahrs.est_grav.x;
   if (MAG <> _NONE_) then
   begin
     ahrs.est_mag.z :=  scos(deltaGyroAngle) * ahrs.est_mag.z - ssin(deltaGyroAngle) * ahrs.est_mag.x;
     ahrs.est_mag.x :=  ssin(deltaGyroAngle) * ahrs.est_mag.z + scos(deltaGyroAngle) * ahrs.est_mag.x;
   end;
   // Calculate Euler Angles
   // TODO: Move it into mavling as we are not using Angles for flight
   ahrs.eul_ref_roll  := arctan2(ahrs.est_grav.y, ahrs.est_grav.z);
   ahrs.eul_ref_pitch := arctan2(ahrs.est_grav.x, ahrs.est_grav.z);
   if (MAG <> _NONE_) then
   begin
     ahrs.eul_ref_yaw := arctan2(ahrs.est_grav.y * ahrs.est_mag.z - ahrs.est_grav.z * ahrs.est_mag.y, ahrs.est_grav.z * ahrs.est_mag.x - ahrs.est_grav.x * ahrs.est_mag.z);
   end;


   // Calculate level reference and apply trims
   inv_mag := 1.0 / Sqrt(ahrs.est_grav.x * ahrs.est_grav.x + ahrs.est_grav.y * ahrs.est_grav.y + ahrs.est_grav.z * ahrs.est_grav.z) * 2000.0;
   ahrs.ctrl_ref.roll  := round(ahrs.est_grav.y * inv_mag);
   ahrs.ctrl_ref.pitch := round(ahrs.est_grav.x * inv_mag);

   //
   inv_mag := 1.0 / Sqrt(ahrs.est_grav.x * ahrs.est_grav.x + ahrs.est_grav.y * ahrs.est_grav.y + ahrs.est_grav.z * ahrs.est_grav.z);
   ahrs.est_grav.x *= inv_mag;
   ahrs.est_grav.y *= inv_mag;
   ahrs.est_grav.z *= inv_mag;
   //
   inv_mag := 1.0 / Sqrt(ahrs.est_mag.x * ahrs.est_mag.x + ahrs.est_mag.y * ahrs.est_mag.y + ahrs.est_mag.z * ahrs.est_mag.z);
   ahrs.est_mag.x *= inv_mag;
   ahrs.est_mag.y *= inv_mag;
   ahrs.est_mag.z *= inv_mag;


   //G x [0, 0, 1]

   ax := ahrs.est_grav.y;
   ay := ahrs.est_grav.x;
   cs := ahrs.est_grav.z;
   si := Sqrt(ahrs.est_grav.x * ahrs.est_grav.x + ahrs.est_grav.y * ahrs.est_grav.y);

   ahrs.eul_ref3_yaw := arctan2(- ax * ay * (1 - cs), 1 - (ay * ay) * (1 - cs));
   ahrs.eul_ref3_pitch := arcsin( ay * si);
   ahrs.eul_ref3_roll := arctan2(ax * si, 1 - (ax *ax) * (1 - cs));

end;

procedure rotate_acc_vector();
var
  deltaGyroAngle: float;
  inv_mag: float;
begin
  // Rotate ACC vector(s), ROLL
   deltaGyroAngle  := (imu.gyro_ahrs.eul.roll) * GYRO_SCALE;
   imu.acc.fr.y :=   cos(deltaGyroAngle) * imu.acc.fr.y + sin(deltaGyroAngle) * imu.acc.fr.z;
   imu.acc.fr.z :=  -sin(deltaGyroAngle) * imu.acc.fr.y + cos(deltaGyroAngle) * imu.acc.fr.z;
   // Rotate ACC vector(s), PITCH
   deltaGyroAngle  := (imu.gyro_ahrs.eul.pitch) * GYRO_SCALE;
   imu.acc.fr.z :=  cos(deltaGyroAngle) * imu.acc.fr.z - sin(deltaGyroAngle) * imu.acc.fr.x;
   imu.acc.fr.x :=  sin(deltaGyroAngle) * imu.acc.fr.z + cos(deltaGyroAngle) * imu.acc.fr.x;
   // Rotate ACC vector(s), YAW
   deltaGyroAngle  := -(imu.gyro_ahrs.eul.yaw) * GYRO_SCALE;
   imu.acc.fr.x := cos(deltaGyroAngle) * imu.acc.fr.x - sin(deltaGyroAngle) * imu.acc.fr.y;
   imu.acc.fr.y := sin(deltaGyroAngle) * imu.acc.fr.x + cos(deltaGyroAngle) * imu.acc.fr.y;
   inv_mag := 255.0 / Sqrt(imu.acc.fr.x * imu.acc.fr.x + imu.acc.fr.y * imu.acc.fr.y + imu.acc.fr.z * imu.acc.fr.z);
   imu.acc.fr.x *= inv_mag;
   imu.acc.fr.y *= inv_mag;
   imu.acc.fr.z *= inv_mag;
end;


procedure apply_acc_cf();
var
  inv_mag: float;
begin
  inv_mag := 1.0 / Sqrt(ahrs.acc_grav.x * ahrs.acc_grav.x + ahrs.acc_grav.y * ahrs.acc_grav.y + ahrs.acc_grav.z * ahrs.acc_grav.z);
  ahrs.est_grav.x := (ahrs.est_grav.x * GYR_CMPF_FACTOR + ahrs.acc_grav.x * inv_mag) * INV_GYR_CMPF_FACTOR;
  ahrs.est_grav.y := (ahrs.est_grav.y * GYR_CMPF_FACTOR + ahrs.acc_grav.y * inv_mag) * INV_GYR_CMPF_FACTOR;
  ahrs.est_grav.z := (ahrs.est_grav.z * GYR_CMPF_FACTOR + ahrs.acc_grav.z * inv_mag) * INV_GYR_CMPF_FACTOR;
  // Error
  ahrs.acc_err.x += ((ahrs.acc_grav.z * inv_mag) * ahrs.est_grav.y - (ahrs.acc_grav.y * inv_mag) * ahrs.est_grav.z)*25.6;
  ahrs.acc_err.y += ((ahrs.acc_grav.z * inv_mag) * ahrs.est_grav.x - (ahrs.acc_grav.x * inv_mag) * ahrs.est_grav.z)*25.6;
  ahrs.acc_err.z -= ((ahrs.acc_grav.x * inv_mag) * ahrs.est_grav.y - (ahrs.acc_grav.y * inv_mag) * ahrs.est_grav.x)*25.6;
end;

procedure pre_filter_acc();
begin
  ahrs.acc_grav.x := ahrs.acc_grav.x * (1.0 - INV_LPF_CONST) + imu.acc.fr.x * INV_LPF_CONST;
  ahrs.acc_grav.y := ahrs.acc_grav.y * (1.0 - INV_LPF_CONST) + imu.acc.fr.y * INV_LPF_CONST;
  ahrs.acc_grav.z := ahrs.acc_grav.z * (1.0 - INV_LPF_CONST) + imu.acc.fr.z * INV_LPF_CONST;
end;

procedure ahrs_reset();
begin
   ahrs.est_grav.x := 0;
   ahrs.est_grav.y := 0;
   ahrs.est_grav.z := 1.0;
   ahrs.acc_grav.x := 255;
   ahrs.acc_grav.y := 0;
   ahrs.acc_grav.z := 0; //imu.acc_1g
   ahrs.acc_err.x := 0;
   ahrs.acc_err.y := 0;
   ahrs.acc_err.z := 0;
   ahrs.est_mag.x := 1.0;
   ahrs.est_mag.y := 0;
   ahrs.est_mag.z := 0;
end;

procedure AHRS_loop_acc();
begin
  pre_filter_acc();
  //apply_acc_cf();
  if (MAG <> _NONE_) then
  begin
    //pre_filter_mag();
    //apply_mag_cf();
  end;
end;

procedure AHRS_Init();
begin
  ahrs_reset();
end;

procedure AHRS_loop_outer();
begin
  rotate_vectors();
  integrate_rotation();
  integrate_euler();
  extract_euler();
end;

end.

