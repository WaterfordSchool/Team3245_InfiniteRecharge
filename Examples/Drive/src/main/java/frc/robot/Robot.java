/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick driver;      //line 27 can be done here as well
  //private Joystick operator;  //uncomment if neccessary

  @Override
  public void robotInit() {
    m_myRobot = new DifferentialDrive(new Talon(0), new Talon(1));  //usually use WPI_TalonSRX or Spark for competition
    driver = new Joystick(0);                                       //Joystick 0 is just the top port on driver station, can change by drag-and-drop or changing the value
    //operator = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(driver.getRawAxis(1), driver.getRawAxis(3));
  }
}
