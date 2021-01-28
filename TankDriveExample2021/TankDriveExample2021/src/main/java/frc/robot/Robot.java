/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  
  /*Begin instantiations at the beggining of class*/

  //Motor Controllers
  Talon leftMotor = new Talon(0);
  Talon rightMotor = new Talon(1);
  
  //Controller
  Joystick driver = new Joystick(0);

  //DifferentialDrive
  DifferentialDrive driveTrain = new DifferentialDrive(leftMotor, rightMotor);

  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.tankDrive(driver.getRawAxis(1), driver.getRawAxis(3));
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
