// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <math.h>

#include <frc/AnalogGyro.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/Joystick.h>
#include <frc/PWMSparkMax.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/velocity.h>

// Create our feedforward gain constants (from the characterization
// tool). Note that these need to have correct units.
static constexpr auto KvLinear = 1.98_V / 1_mps;
static constexpr auto KaLinear = 0.2_V / 1_mps_sq;
static constexpr auto KvAngular = 1.5_V / 1_rad_per_s;
static constexpr auto KaAngular = 0.3_V / 1_rad_per_s_sq;

frc::Field2d m_field;
frc::AnalogGyro m_gyro{1};
frc::sim::AnalogGyroSim m_gyroSim{m_gyro};
// Create the simulation model of our drivetrain.
frc::sim::DifferentialDrivetrainSim m_driveSim{
  frc::DCMotor::NEO(2), // 2 NEO motors on each side of the drivetrain.
  7.29,               // 7.29:1 gearing reduction.
  7.5_kg_sq_m,        // MOI of 7.5 kg m^2 (from CAD model).
  60_kg,              // The mass of the robot is 60 kg.
  3_in,               // The robot uses 3" radius wheels.
  0.7112_m,           // The track width is 0.7112 meters.

  // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}};
class steerpid{
  public:
  double p,i,d;
  double integral;
  double previousinput;
  steerpid(double pn,double in,double dn,double val) {previousinput=val;integral=0;p=pn;i=in;d=dn; }

  double calc(double error){
    double power=error*p;
    if ((integral>0) && (error*i<0))
      integral=0;
    if((integral<0) && (error*i>0))
      integral=0;
    integral += error*i;
    if ((-20<error) && (error<20))
      power += integral;
    else 
      integral=0;
    power += (error-previousinput)*d;
    previousinput=error;
    return power;
  }
};
class JoystickPower{
  public:
  double ary[8][2]={
    {-1,-12},
    {-.75,-1},
    {-.5,-.2},
    {-.25,0},
    {.25,0},
    {.5,.2},
    {.75,1},
    {1,12}
  };
  double interp(double joy){
    if(joy<=ary[0][0]) return(ary[0][1]);
    if(joy>=ary[7][0]) return(ary[7][1]);
    for(int i=0;i<7;i++)
      if((joy>=ary[i+0][0]) && (joy<=ary[i+1][0])) return(joy-ary[i+0][0])*(ary[i+1][1]-ary[i+0][1])/(ary[i+1][0]-ary[i+0][0])+ary[i+0][1];
    return 0;
  }
};
/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a
 * robot drive straight. This program uses a joystick to drive forwards and
 * backwards while the gyro is used for direction keeping.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override { m_gyro.SetSensitivity(kVoltsPerDegreePerSecond); 
  frc::SmartDashboard::PutData("Field",&m_field);
  frc::Pose2d pose(units::length::meter_t(5),units::length::meter_t(5),frc::Rotation2d(units::degree_t(180)));
  m_driveSim.SetPose(pose);
     m_field.SetRobotPose(m_driveSim.GetPose());
     m_joystick.SetTwistChannel(4);
  }
  JoystickPower jp;

  /**
   * The motor speed is set from the joystick while the DifferentialDrive
   * turning value is assigned from the error between the setpoint and the gyro
   * angle.
   */
  void TeleopPeriodic() override {
    double turningValue = (kAngleSetpoint - m_gyro.GetAngle()) * kP;
    double x,y,z,angle;
    // Invert the direction of the turn if we are going backwards
    turningValue = std::copysign(turningValue, m_joystick.GetY());
    m_robotDrive.ArcadeDrive(m_joystick.GetY(), turningValue);
    //printf("running: %f %f %f: ",x=m_joystick.GetX(),y=m_joystick.GetY(),z=m_joystick.GetZ());
    x=m_joystick.GetX(),y=m_joystick.GetY(),z=m_joystick.GetZ();
    double max=fmax(fmax(x,y),z);
    double min=fmin(fmin(x,y),z);
    double mid;
    double section;
    if((x<=y) && (y<=z)) {
      section=1;
       mid=y;
      angle=60-(mid-min)/(max-min)*60;
    }
    if((y<=x) && (x<=z)) {
      section=2;
       mid=x;
      angle=60+(mid-min)/(max-min)*60;
    }
    if((y<=z) && (z<=x)) {
      section=3;
       mid=z;
      angle=180-(mid-min)/(max-min)*60;
    }
    if((z<=y) && (y<=x)) {
      section=4;
       mid=y;
      angle=180+(mid-min)/(max-min)*60;
    }
    if((z<=x) && (x<=y)) {
      section=5;
       mid=x;
      angle=300-(mid-min)/(max-min)*60;
    }
    if((x<=z) && (z<=y)) {
      section=6;
       mid=z;
      angle=300+(mid-min)/(max-min)*60;
    }
    //angle=360-angle;
    printf("\nJoy_angle: %.1f, Robot_angle: %.1f, ",angle,(double)m_driveSim.GetHeading().Degrees());
    // printf("sml: %f %f %f %f: \n", min, mid, max, section);


    // Move the robot in simulation
    // Set the inputs to the system. Note that we need to convert
  // the [-1, 1] PWM signal to voltage by multiplying it by the
  // robot controller voltage.
  double left,right;
  left = right = -jp.interp(m_joystick.GetTwist());

  double delta_angle = angle - (double)m_driveSim.GetHeading().Degrees();
  //delta_angle = (delta_angle % 360) - 180; #python
  delta_angle = fmod(360 + fmod(delta_angle+180,360),360)-180; // c++

  double steer=fmax(-12,fmin(12,m_steerpid.calc(delta_angle)));
  left -= steer;
  right += steer;
  printf("d=% 05.1f, ",delta_angle);
  printf("steer=%.3f, ",steer);
  m_driveSim.SetInputs( units::volt_t(left),units::volt_t(right));

     // Advance the model by 20 ms. Note that if you are running this
  // subsystem in a separate thread or have changed the nominal timestep
  // of TimedRobot, this value needs to match it.
  m_driveSim.Update(20_ms);


    //m_gyroSim[0].SetAngle(-m_driveSim.GetHeading().Degrees());
    m_field.SetRobotPose(m_driveSim.GetPose());
  }

 private:
  static constexpr double kAngleSetpoint = 0.0;
  static constexpr double kP = 0.005;  // Proportional turning constant

  // Gyro calibration constant, may need to be adjusted. Gyro value of 360 is
  // set to correspond to one full revolution.
  static constexpr double kVoltsPerDegreePerSecond = 0.0128;

  static constexpr int kLeftMotorPort = 0;
  static constexpr int kRightMotorPort = 1;
  static constexpr int kGyroPort = 0;
  static constexpr int kJoystickPort = 0;

  frc::PWMSparkMax m_left{kLeftMotorPort};
  frc::PWMSparkMax m_right{kRightMotorPort};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};

  frc::AnalogGyro m_gyro{kGyroPort};
  frc::Joystick m_joystick{kJoystickPort};
  steerpid m_steerpid{0.4,0.001,3.2,0};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
