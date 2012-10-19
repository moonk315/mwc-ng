unit FMain;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, OpenGLContext, Forms, Controls, Graphics,
  Dialogs, ExtCtrls, StdCtrls, GL, GLU;

type

  { TForm1 }

  TForm1 = class(TForm)
    AccErrY: TLabeledEdit;
    AccErrZ: TLabeledEdit;
    EdtPitch1: TLabeledEdit;
    EdtPitch2: TLabeledEdit;
    EdtPitch3: TLabeledEdit;
    EdtRoll1: TLabeledEdit;
    EdtRoll2: TLabeledEdit;
    EdtRoll3: TLabeledEdit;
    EdtYaw: TLabeledEdit;
    EdtRoll: TLabeledEdit;
    EdtPitch: TLabeledEdit;
    AccErrX: TLabeledEdit;
    EdtYaw1: TLabeledEdit;
    EdtYaw2: TLabeledEdit;
    EdtYaw3: TLabeledEdit;
    Label1: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    OpenGLControl1: TOpenGLControl;
    OpenGLControl2: TOpenGLControl;
    OpenGLControl3: TOpenGLControl;
    tmAcc: TTimer;
    TmUpdate: TTimer;
    TmOuter: TTimer;
    procedure FormCreate(Sender: TObject);
    procedure OpenGLControl1Paint(Sender: TObject);
    procedure OpenGLControl2Paint(Sender: TObject);
    procedure OpenGLControl3Paint(Sender: TObject);
    procedure tmAccTimer(Sender: TObject);
    procedure TmOuterTimer(Sender: TObject);
    procedure TmUpdateTimer(Sender: TObject);
  private
    procedure OpenGLControlInit;
    procedure OpenGLControlPainCube;
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
  imu.gyro_ahrs.eul.roll := 60*255;
  imu.gyro_ahrs.eul.pitch := 60*255;
  imu.gyro_ahrs.eul.yaw := 0;
  q.v.Z := 0;
  q.W := 1;
end;

procedure TForm1.OpenGLControlInit;
begin
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, double(OpenGLControl1.width) / OpenGLControl1.height, 0.1, 100.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0, 0.0,-6.0);
  glLineWidth(2.0);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
end;

procedure TForm1.OpenGLControlPainCube;
begin
  glBegin(GL_QUADS);
          glColor3f(0.0,1.0,0.0);                              // Set The Color To Green
          glVertex3f( 1.0, 1.0,-1.0);                  // Top Right Of The Quad (Top)
          glVertex3f(-1.0, 1.0,-1.0);                  // Top Left Of The Quad (Top)
          glVertex3f(-1.0, 1.0, 1.0);                  // Bottom Left Of The Quad (Top)
          glVertex3f( 1.0, 1.0, 1.0);                  // Bottom Right Of The Quad (Top)
  glEnd();
  glBegin(GL_QUADS);
          glColor3f(1.0,0.5,0.0);                              // Set The Color To Orange
          glVertex3f( 1.0,-1.0, 1.0);                  // Top Right Of The Quad (Bottom)
          glVertex3f(-1.0,-1.0, 1.0);                  // Top Left Of The Quad (Bottom)
          glVertex3f(-1.0,-1.0,-1.0);                  // Bottom Left Of The Quad (Bottom)
          glVertex3f( 1.0,-1.0,-1.0);                  // Bottom Right Of The Quad (Bottom)
  glEnd();
  glBegin(GL_QUADS);
          glColor3f(1.0,0.0,0.0);                              // Set The Color To Red
          glVertex3f( 1.0, 1.0, 1.0);                  // Top Right Of The Quad (Front)
          glVertex3f(-1.0, 1.0, 1.0);                  // Top Left Of The Quad (Front)
          glVertex3f(-1.0,-1.0, 1.0);                  // Bottom Left Of The Quad (Front)
          glVertex3f( 1.0,-1.0, 1.0);                  // Bottom Right Of The Quad (Front)
  glEnd();
  glBegin(GL_QUADS);
          glColor3f(1.0,1.0,0.0);                              // Set The Color To Yellow
          glVertex3f( 1.0,-1.0,-1.0);                  // Bottom Left Of The Quad (Back)
          glVertex3f(-1.0,-1.0,-1.0);                  // Bottom Right Of The Quad (Back)
          glVertex3f(-1.0, 1.0,-1.0);                  // Top Right Of The Quad (Back)
          glVertex3f( 1.0, 1.0,-1.0);                  // Top Left Of The Quad (Back)
  glEnd();
  glBegin(GL_QUADS);
          glColor3f(0.0,0.0,1.0);                              // Set The Color To Blue
          glVertex3f(-1.0, 1.0, 1.0);                  // Top Right Of The Quad (Left)
          glVertex3f(-1.0, 1.0,-1.0);                  // Top Left Of The Quad (Left)
          glVertex3f(-1.0,-1.0,-1.0);                  // Bottom Left Of The Quad (Left)
          glVertex3f(-1.0,-1.0, 1.0);                  // Bottom Right Of The Quad (Left)
  glEnd();
  glBegin(GL_QUADS);
          glColor3f(1.0,0.0,1.0);                              // Set The Color To Violet
          glVertex3f( 1.0, 1.0,-1.0);                  // Top Right Of The Quad (Right)
          glVertex3f( 1.0, 1.0, 1.0);                  // Top Left Of The Quad (Right)
          glVertex3f( 1.0,-1.0, 1.0);                  // Bottom Left Of The Quad (Right)
          glVertex3f( 1.0,-1.0,-1.0);                  // Bottom Right Of The Quad (Right)
  glEnd();
end;

procedure TForm1.OpenGLControl1Paint(Sender: TObject);
var
  Speed, c1, c2, c3, s1, s2, s3, x, y, z, angle: Double;
begin
  OpenGLControlInit;
  glPushMatrix();

  c1 := cos(ahrs.eul_ref_yaw / 2);
  c2 := cos(ahrs.eul_ref_pitch / 2);
  c3 := cos(ahrs.eul_ref_roll / 2);
  s1 := sin(ahrs.eul_ref_yaw / 2);
  s2 := sin(ahrs.eul_ref_pitch / 2);
  s3 := sin(ahrs.eul_ref_roll / 2);
  angle := 2 * arccos(c1 * c2 * c3 - s1 * s2 * s3);
  x := s1 * s2 * c3 +c1 * c2 * s3;
  y := s1 * c2 * c3 + c1 * s2 * s3;
  z := c1 * s2 * c3 - s1 * c2 * s3;
  glRotatef(radtodeg(angle), x, y, -z);

  OpenGLControlPainCube;
  glPopMatrix();

  glBegin(GL_LINES);
    glColor3f(0.0,0.0,0.0);
    glVertex3f(0, 0, 0);
    glVertex3f(ahrs.est_grav.X, ahrs.est_grav.Z, ahrs.est_grav.X);
  glEnd();

  glBegin(GL_LINES);
    glColor3f(100.0,0.0,0.0);
    glVertex3f(0, 0, 0);
    glVertex3f(ahrs.est_mag.X, ahrs.est_mag.Z, ahrs.est_mag.X);
  glEnd();

  OpenGLControl1.SwapBuffers;
end;

procedure TForm1.OpenGLControl2Paint(Sender: TObject);
var
  Speed, c1, c2, c3, s1, s2, s3, x, y, z, angle: Double;
begin
  OpenGLControlInit;

  glPushMatrix();

  c1 := cos(ahrs.eul_ref2_yaw / 2);
  c2 := cos(ahrs.eul_ref2_pitch / 2);
  c3 := cos(ahrs.eul_ref2_roll / 2);
  s1 := sin(ahrs.eul_ref2_yaw / 2);
  s2 := sin(ahrs.eul_ref2_pitch / 2);
  s3 := sin(ahrs.eul_ref2_roll / 2);

  angle := 2 * arccos(c1 * c2 * c3 - s1 * s2 * s3);
  x := s1 * s2 * c3 +c1 * c2 * s3;
  y := s1 * c2 * c3 + c1 * s2 * s3;
  z := c1 * s2 * c3 - s1 * c2 * s3;

  glRotatef(radtodeg(angle), x, y, -z);

  OpenGLControlPainCube;

  glPopMatrix();
  glBegin(GL_LINES);
    glColor3f(0.0,0.0,0.0);
    glVertex3f(0, 0, 0);
    glVertex3f(ahrs.est_grav.X, ahrs.est_grav.Z, ahrs.est_grav.X);
  glEnd();


  OpenGLControl2.SwapBuffers;
end;

procedure TForm1.OpenGLControl3Paint(Sender: TObject);
var
  Speed, c1, c2, c3, s1, s2, s3, x, y, z, angle: Double;
begin
  OpenGLControlInit;
  glPushMatrix();

  c1 := cos(ahrs.eul_ref3_yaw / 2);
  c2 := cos(ahrs.eul_ref3_pitch / 2);
  c3 := cos(ahrs.eul_ref3_roll / 2);
  s1 := sin(ahrs.eul_ref3_yaw / 2);
  s2 := sin(ahrs.eul_ref3_pitch / 2);
  s3 := sin(ahrs.eul_ref3_roll / 2);

  angle := 2 * arccos(c1 * c2 * c3 - s1 * s2 * s3);
  x := s1 * s2 * c3 +c1 * c2 * s3;
  y := s1 * c2 * c3 + c1 * s2 * s3;
  z := c1 * s2 * c3 - s1 * c2 * s3;

  glRotatef(radtodeg(angle), x, y, -z);

  OpenGLControlPainCube;

  glPopMatrix();
  glBegin(GL_LINES);
    glColor3f(0.0,0.0,0.0);
    glVertex3f(0, 0, 0);
    glVertex3f(ahrs.est_grav.X, ahrs.est_grav.Z, ahrs.est_grav.X);
  glEnd();

  OpenGLControl3.SwapBuffers;
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
  EdtYaw.Text := FormatFloat('0.000', radtodeg(ahrs.eul_ref_yaw));

  EdtRoll1.Text := FormatFloat('0.000', radtodeg(ahrs.eul_ref2_roll));
  EdtPitch1.Text := FormatFloat('0.000', radtodeg(ahrs.eul_ref2_pitch));
  EdtYaw1.Text := FormatFloat('0.000', radtodeg(ahrs.eul_ref2_yaw));

  EdtRoll2.Text := FormatFloat('0.000',  abs(radtodeg(ahrs.eul_ref2_roll)  - radtodeg(ahrs.eul_ref_roll)));
  EdtPitch2.Text := FormatFloat('0.000', abs(radtodeg(ahrs.eul_ref2_pitch) - radtodeg(ahrs.eul_ref_pitch)));
  EdtYaw2.Text := FormatFloat('0.000',   abs(radtodeg(ahrs.eul_ref2_yaw)   - radtodeg(ahrs.eul_ref_yaw)));


  EdtRoll3.Text := FormatFloat('0.000', radtodeg(ahrs.eul_ref3_roll));
  EdtPitch3.Text := FormatFloat('0.000', radtodeg(ahrs.eul_ref3_pitch));
  EdtYaw3.Text := FormatFloat('0.000', radtodeg(ahrs.eul_ref3_yaw));

  AccErrX.Text := FormatFloat('0.000', ahrs.acc_err.X);
  AccErrY.Text := FormatFloat('0.000', ahrs.acc_err.Y);
  AccErrZ.Text := FormatFloat('0.000', ahrs.acc_err.Z);
  OpenGLControl1.Invalidate;
  OpenGLControl2.Invalidate;
  OpenGLControl3.Invalidate;
end;

end.

