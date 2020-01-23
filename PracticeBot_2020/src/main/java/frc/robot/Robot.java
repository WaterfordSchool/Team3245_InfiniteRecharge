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
  Talon rightFront = new Talon(RobotMap.RIGHT_FRONT_MOTOR_ID);
  Talon rightBack = new Talon(RobotMap.RIGHT_BACK_MOTOR_ID);

  Talon leftFront = new Talon(RobotMap.LEFT_FRONT_MOTOR_ID);
  Talon leftBack = new Talon(RobotMap.LEFT_BACK_MOTOR_ID);

  //Speed Controller Groups & differential drive object
  SpeedControllerGroup left = new SpeedControllerGroup(rightFront, rightBack);
  SpeedControllerGroup right = new SpeedControllerGroup(leftFront, leftBack);

  DifferentialDrive dT = new DifferentialDrive(left, right);

  //Joystick
  Joystick driver = new Joystick(0);
  double leftStickVal; //Assigned in teleopPeriodic
  double rightStickVal; //Assigned in teleopPeriodic

  //four sims, standard pwm talon controllers




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

  @Override
  public void teleopPeriodic() {
    //Assigns values because it needs to be routinely updated
    leftStickVal = driver.getRawAxis(RobotMap.LEFT_AXIS_ID);
    rightStickVal = driver.getRawAxis(RobotMap.RIGHT_AXIS_ID);

    //Tank drive method call
    dT.tankDrive(leftStickVal * RobotMap.DRIVE_SPEED_ID, rightStickVal * RobotMap.DRIVE_SPEED_ID);
  }

  @Override
  public void testPeriodic() {
  }
}
