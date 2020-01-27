/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Motors
  Talon right = new Talon(RobotMap.RIGHT_FRONT_MOTOR_ID);
  //Talon rightBack = new Talon(RobotMap.RIGHT_BACK_MOTOR_ID);

  Talon left = new Talon(RobotMap.LEFT_FRONT_MOTOR_ID);
  //Talon leftBack = new Talon(RobotMap.LEFT_BACK_MOTOR_ID);

  Talon flywheelMotor = new Talon(RobotMap.FLYWHEEL_MOTOR_ID);
  Talon indexMotor = new Talon(RobotMap.INDEX_MOTOR_ID);
/**
  //Speed Controller Groups & differential drive object
  SpeedControllerGroup left = new SpeedControllerGroup(rightFront, rightBack);
  SpeedControllerGroup right = new SpeedControllerGroup(leftFront, leftBack);
*/
  DifferentialDrive dT = new DifferentialDrive(left, right);

  //Joystick
  Joystick driver = new Joystick(RobotMap.JOYSTICK_DRIVER_PORT_ID); //driver joystick
  Joystick operator = new Joystick(RobotMap.JOYSTICK_OPERATOR_PORT_ID); //operator joystick
  double leftStickVal; //Assigned in teleopPeriodic
  double rightStickVal; //Assigned in teleopPeriodic

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  public void index(){
    if(operator.getRawButton(RobotMap.INDEX_BUTTON_ID)){
      indexMotor.set(RobotMap.INDEX_SPEED_ID);
    }
    else{
      indexMotor.set(0.0);
    }
  }

  public void flyWheel(){
    if(operator.getRawButton(RobotMap.FLYWHEEL_BUTTON_ID)){
      flywheelMotor.set(RobotMap.FLYWHEEL_SPEED_ID);
    }
    else{
      flywheelMotor.set(0.0);
    }
  }

  @Override
  public void teleopPeriodic() {
    //Assigns values because it needs to be routinely updated
    leftStickVal = driver.getRawAxis(RobotMap.LEFT_AXIS_ID);
    rightStickVal = driver.getRawAxis(RobotMap.RIGHT_AXIS_ID);

    //Tank drive method call
    dT.tankDrive(-leftStickVal * RobotMap.DRIVE_SPEED_ID, -rightStickVal * RobotMap.DRIVE_SPEED_ID);

    index();
    flyWheel();
  }

  @Override
  public void testPeriodic() {
  }
}
