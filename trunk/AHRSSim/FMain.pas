unit FMain;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, Forms, Controls, Graphics, Dialogs, ExtCtrls;

type

  { TForm1 }

  TForm1 = class(TForm)
    AccErrY: TLabeledEdit;
    AccErrZ: TLabeledEdit;
    EdtRoll: TLabeledEdit;
    EdtPitch: TLabeledEdit;
    AccErrX: TLabeledEdit;
    tmAcc: TTimer;
    TmUpdate: TTimer;
    TmOuter: TTimer;
    procedure FormCreate(Sender: TObject);
    procedure tmAccTimer(Sender: TObject);
    procedure TmOuterTimer(Sender: TObject);
    procedure TmUpdateTimer(Sender: TObject);
  private
    { private declarations }
  public
    { public declarations }
  end;

var
  Form1: TForm1;

implementation

uses UAHRS, Math;

{$R *.lfm}

{ TForm1 }


procedure TForm1.FormCreate(Sender: TObject);
begin
  AHRS_Init();
  imu.acc.fr.x := 255;
  imu.acc.fr.y := 0;
  imu.acc.fr.z := 0;
  imu.gyro_ahrs.eul.roll := 0;
  imu.gyro_ahrs.eul.pitch := 50 * 256;
  imu.gyro_ahrs.eul.yaw := 0;
end;

procedure TForm1.tmAccTimer(Sender: TObject);
begin
  AHRS_loop_acc();
end;

procedure TForm1.TmOuterTimer(Sender: TObject);
begin
  AHRS_loop_outer();
  rotate_acc_vector();
end;

procedure TForm1.TmUpdateTimer(Sender: TObject);
begin
  EdtRoll.Text := FormatFloat('0.000', radtodeg(ahrs.eul_ref_roll));
  EdtPitch.Text := FormatFloat('0.000', radtodeg(ahrs.eul_ref_pitch));

  AccErrX.Text := FormatFloat('0.000', ahrs.acc_err.X);
  AccErrY.Text := FormatFloat('0.000', ahrs.acc_err.Y);
  AccErrZ.Text := FormatFloat('0.000', ahrs.acc_err.Z);
end;

end.

