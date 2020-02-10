/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "WPILibVersion.h"
#include <iostream>
#include <ctime>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Talon.h>
#include <frc/Joystick.h>
#include <frc/RobotDrive.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <cameraserver/CameraServer.h>

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
using namespace frc;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  frc::Joystick controller{0};  //Joystick 0
  //Joystick controller2{1};

  frc::Talon lf{0}; // Left Front Wheel 1
	frc::Talon rf{1}; // Right Front Wheel 2
	frc::Talon lr{2}; // Left Rear Wheel 3
	frc::Talon rr{3}; // Right Rear Wheel 4
  frc::RobotDrive drive{lf, lr, rf, rr};
  
 /* frc::Talon ar{5}; // Arm Right Motor
  frc::Talon al{4}; // Arm Left Motor

  // These two mototrs act as one for the raising and lowering of the lift system.
  frc::Talon lift_motor1{6};
  frc::Talon lift_motor2{7};
  bool lift_state = false; // Keeps track of which position the lift is at.
  clock_t lift_timer;

  // frc::Talon compressor{8};  Compressor
  frc::Compressor c{0};

  // Solenoids for hatch pistons.
  frc::DoubleSolenoid hatch_piston_1{0, 1};
  frc::DoubleSolenoid hatch_piston_2{2, 3};
*/
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
 /* rr.SetInverted(true);
  rf.SetInverted(true);

  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);

  hatch_piston_1.Set(frc::DoubleSolenoid::Value::kReverse);
  hatch_piston_2.Set(frc::DoubleSolenoid::Value::kReverse);

  c.SetClosedLoopControl(true); */
}



void Robot::TeleopInit() {
  //rf.SetInverted(true);
 // lf.SetInverted(true);

 /* frc::CameraServer::GetInstance()->StartAutomaticCapture(0);

  hatch_piston_1.Set(frc::DoubleSolenoid::Value::kReverse);
  hatch_piston_2.Set(frc::DoubleSolenoid::Value::kReverse);

  c.SetClosedLoopControl(true); */
}

void Robot::TeleopPeriodic() {
  frc::Joystick controller{0};  
  double x, y, z;
  double factor = 0.68;

  x = (controller.GetRawAxis(0)) * factor; //Move Fowards or Backwards
	y = (controller.GetRawAxis(1)) * factor; //Strafe <- or ->
	z = (controller.GetRawAxis(4)) * factor; //Turn <- or ->

  drive.MecanumDrive_Cartesian(x, y, z, 0);

  // lift_motor1.Set(y);
  // lift_motor2.Set(y);

  /*if(controller.GetRawButton(1)){ //ball intake 
    ar.Set(-1);
    al.Set(1);

    clock_t now = clock();
    while ((clock() - now) < (CLOCKS_PER_SEC / 2));

    ar.Set(0);
    al.Set(0);
  }
  if(controller.GetRawButton(2)){ //ball outake
    ar.Set(1);
    al.Set(-1);

    clock_t now = clock();
    while ((clock() - now) < (CLOCKS_PER_SEC / 2));

    ar.Set(0);
    al.Set(0);
  }
  if (controller.GetRawButton(3)) {
    lift_motor1.Set(0.3);
    lift_motor2.Set(0.3);
    lift_timer = clock();
    lift_state = true;
    
  } else if (controller.GetRawButton(4)) {
    lift_motor1.Set(-0.3);
    lift_motor2.Set(-0.3);
    lift_timer = clock();
    lift_state = true;
  }
  if ( (lift_state) && ((clock() - lift_timer) >= (CLOCKS_PER_SEC / 8) ) )  {
    lift_motor1.Set(0);
    lift_motor2.Set(0);
    lift_state = false;
  }
  

  Trigger, extends and retracts pistons.
  if (controller.GetRawButton(5)) {
    hatch_piston_1.Set(frc::DoubleSolenoid::Value::kForward);
    hatch_piston_2.Set(frc::DoubleSolenoid::Value::kForward);
  }
  if(controller.GetRawButton(6)){
    hatch_piston_1.Set(frc::DoubleSolenoid::Value::kReverse);
    hatch_piston_2.Set(frc::DoubleSolenoid::Value::kReverse);
  } */
}

void Robot::AutonomousPeriodic() {
  Robot::TeleopPeriodic();  
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

