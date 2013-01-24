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
 // ahrs.eul_ref3_roll += imu.gyro_ahrs.eul.roll * GYRO_SCALE;
 // ahrs.eul_ref3_pitch += imu.gyro_ahrs.eul.pitch * GYRO_SCALE;
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
  inv_mag, tmp: float;
begin
  // Rotate Estimated vector(s), YAW
  deltaGyroAngle  := -(imu.gyro_ahrs.eul.yaw - ahrs.acc_err.z) * GYRO_SCALE;
  tmp := ahrs.est_grav.x;
  ahrs.est_grav.x :=  scos(deltaGyroAngle) * tmp - ssin(deltaGyroAngle) * ahrs.est_grav.y;
  ahrs.est_grav.y :=  ssin(deltaGyroAngle) * tmp + scos(deltaGyroAngle) * ahrs.est_grav.y;
  if (MAG <> _NONE_) then
  begin
    tmp := ahrs.est_mag.x;
    ahrs.est_mag.x :=  scos(deltaGyroAngle) * tmp - ssin(deltaGyroAngle) * ahrs.est_mag.y;
    ahrs.est_mag.y :=  ssin(deltaGyroAngle) * tmp + scos(deltaGyroAngle) * ahrs.est_mag.y;
  end;
  // Rotate Estimated vector(s), PITCH
  deltaGyroAngle  := (imu.gyro_ahrs.eul.pitch - ahrs.acc_err.y) * GYRO_SCALE;
  tmp := ahrs.est_grav.x;
  ahrs.est_grav.x :=  ssin(deltaGyroAngle) * ahrs.est_grav.z + scos(deltaGyroAngle) * tmp;
  ahrs.est_grav.z :=  scos(deltaGyroAngle) * ahrs.est_grav.z - ssin(deltaGyroAngle) * tmp;
  if (MAG <> _NONE_) then
  begin
    tmp := ahrs.est_mag.x;
    ahrs.est_mag.x :=  ssin(deltaGyroAngle) * ahrs.est_mag.z + scos(deltaGyroAngle) * tmp;
    ahrs.est_mag.z :=  scos(deltaGyroAngle) * ahrs.est_mag.z - ssin(deltaGyroAngle) * tmp;
  end;
  // Rotate Estimated vector(s), ROLL
   deltaGyroAngle  := -(imu.gyro_ahrs.eul.roll - ahrs.acc_err.x) * GYRO_SCALE;
   tmp := ahrs.est_grav.y;
   ahrs.est_grav.y :=  scos(deltaGyroAngle) * tmp - ssin(deltaGyroAngle) * ahrs.est_grav.z;
   ahrs.est_grav.z :=  ssin(deltaGyroAngle) * tmp + scos(deltaGyroAngle) * ahrs.est_grav.z;
   if (MAG <> _NONE_) then
   begin
     tmp := ahrs.est_mag.y;
     ahrs.est_mag.y :=  scos(deltaGyroAngle) * tmp - ssin(deltaGyroAngle) * ahrs.est_mag.z;
     ahrs.est_mag.z :=  ssin(deltaGyroAngle) * tmp + scos(deltaGyroAngle) * ahrs.est_mag.z;
   end;
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

   // Calculate level reference and apply trims
   inv_mag := 1.0 / Sqrt(ahrs.est_grav.x * ahrs.est_grav.x + ahrs.est_grav.y * ahrs.est_grav.y + ahrs.est_grav.z * ahrs.est_grav.z) * 2000.0;
   ahrs.ctrl_ref.roll  := round(ahrs.est_grav.y * inv_mag);
   ahrs.ctrl_ref.pitch := round(ahrs.est_grav.x * inv_mag);

end;

procedure extract_euler_vect();
var
  _sin,_cos, Mpz, Mfy, Mfx, Mfz, Gpz, invmag: float;
begin
  Gpz :=  ahrs.est_grav.Z;
  ahrs.eul_ref_roll := arctan2(ahrs.est_grav.y, Gpz);
  // To earth frame (roll)
  invmag := 1.0/sqrt(Gpz * Gpz + ahrs.est_grav.Y*ahrs.est_grav.Y);
  _sin := ahrs.est_grav.Y * invmag;
  _cos := Gpz * invmag;
  Gpz := ahrs.est_grav.Y * _sin + Gpz * _cos;
  ahrs.eul_ref_pitch := arctan2(ahrs.est_grav.x, Gpz);
  if (MAG <> _NONE_) then
  begin
    // To earth frame (roll)
    Mpz :=  ahrs.est_mag.Z;
    Mfy := ahrs.est_mag.Y * _cos - Mpz * _sin;
    Mpz := ahrs.est_mag.Y * _sin + Mpz * _cos;
    ahrs.rot_mat[0].cos := _cos;
    ahrs.rot_mat[0].sin := _sin;
    // To earth frame (pitch)
    invmag := 1.0/sqrt(Gpz * Gpz + ahrs.est_grav.X * ahrs.est_grav.X);
    _sin := -ahrs.est_grav.x * invmag;
    _cos := Gpz * invmag;
    if (_cos < 0.0) then _cos := -_cos;
    Mfx :=  ahrs.est_mag.X * _cos + Mpz * _sin;
    //Mfz := -ahrs.est_mag.X * _sin + Mpz * _cos;
    ahrs.eul_ref_yaw := arctan2(-Mfy, Mfx);
    ahrs.rot_mat[1].cos := _cos;
    ahrs.rot_mat[1].sin := _sin;
    // To earth frame (yaw)
    invmag := 1.0/sqrt(Mfy * Mfy + Mfx * Mfx);
    _sin := -Mfy * invmag;
    _cos :=  Mfx * invmag;
    ahrs.rot_mat[2].cos := _cos;
    ahrs.rot_mat[2].sin := _sin;
  end;
end;

procedure _to_earth(const v: TFpVector; var res: TFpVector);
var
  _sin,_cos, Vfy, Vfx, Vfz: float;
begin
  _sin := ahrs.rot_mat[0].sin;
  _cos := ahrs.rot_mat[0].cos;
  Vfz :=  V.Z;
  Vfy := V.Y * _cos - Vfz * _sin;
  Vfz := V.Y * _sin + Vfz * _cos;
  _sin := ahrs.rot_mat[1].sin;
  _cos := ahrs.rot_mat[1].cos;
  Vfx :=  v.X * _cos + Vfz * _sin;
  Vfz := -v.X * _sin + Vfz * _cos;
  _sin := ahrs.rot_mat[2].sin;
  _cos := ahrs.rot_mat[2].cos;
  Vfx :=  Vfx * _cos - Vfy * _sin;
  Vfy :=  Vfx * _sin + Vfy * _cos;
  res.X := Vfx;
  res.Y := Vfy;
  res.Z := Vfz;
end;

procedure to_earth(const v: TFpVector; var res: TFpVector);
var
  _sin,_cos, tmp: float;
begin
  _sin := ahrs.rot_mat[0].sin;
  _cos := ahrs.rot_mat[0].cos;
  res.y := V.Y * _cos - v.z * _sin;
  res.z := V.Y * _sin + v.z * _cos;
  _sin := ahrs.rot_mat[1].sin;
  _cos := ahrs.rot_mat[1].cos;
  res.x :=  v.X * _cos + res.z * _sin;
  res.z := -v.X * _sin + res.z * _cos;
  _sin := ahrs.rot_mat[2].sin;
  _cos := ahrs.rot_mat[2].cos;
  tmp := res.x;
  res.x :=  tmp * _cos - res.y * _sin;
  res.y :=  tmp * _sin + res.y * _cos;
end;


procedure to_local(const v: TFpVector; var res: TFpVector);
var
  _sin,_cos, ry, tmp: float;
begin
  _sin := ahrs.rot_mat[2].sin;
  _cos := ahrs.rot_mat[2].cos;
  res.x :=  v.x * _cos + v.y * _sin;
  res.y :=  -v.x * _sin + v.y * _cos;

  _sin := ahrs.rot_mat[1].sin;
  _cos := ahrs.rot_mat[1].cos;
  tmp := res.X;
  res.x :=  tmp * _cos - v.z * _sin;
  res.z :=  tmp * _sin + v.z * _cos;

  _sin := ahrs.rot_mat[0].sin;
  _cos := ahrs.rot_mat[0].cos;
  tmp := res.y;
  res.y :=  tmp * _cos + res.z * _sin;
  res.z := -tmp * _sin + res.z * _cos;
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
   ahrs.est_mag.z := -1.0;
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
  extract_euler_vect();
  to_earth(ahrs.est_grav, ahrs.grav_earth);
  to_local(ahrs.grav_earth, ahrs.grav_local);
end;

end.

