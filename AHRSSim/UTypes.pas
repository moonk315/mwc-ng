unit UTypes;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils;

type
  float = real;


  TFpVector = record
    X: float;
    Y: float;
    Z: float;
  end;

  TFpQ = record
    v: TFpVector;
    w: float;
  end;

  TGyroData = record
    roll: integer;
    pitch: integer;
    yaw: integer
  end;

  TFrameData = record
    x: {integer}float;
    y: {integer}float;
    z: {integer} float;
  end;

  TEul = record
    eul: TGyroData;
  end;

  TFr = record
    fr: TFrameData;
  end;

  TAHRS = record
    acc_grav: TFpVector;
    est_grav: TFpVector;
    mag_mag: TFpVector;
    est_mag: TFpVector;
    mag_prev: TFpVector;
    inv_mag_prev: TFpVector;
    eul_ref_roll: float;
    eul_ref_pitch: float;
    eul_ref_yaw: float;
    ctrl_ref: TGyroData;
    acc_err: TFpVector;

    eul_ref2_roll: float;
    eul_ref2_pitch: float;
    eul_ref2_yaw: float;

    eul_ref3_roll: float;
    eul_ref3_pitch: float;
    eul_ref3_yaw: float;

  end;


  TIMU = record
    gyro_ahrs: TEul;
    acc: TFr;
  end;


implementation

end.

