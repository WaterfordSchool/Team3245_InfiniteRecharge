/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.SparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  Talon r0 = new Talon(22);
  Talon r1 = new Talon(23);
  Talon r2 = new Talon(24);
  Spark l0 = new Spark(20);
  Spark l1 = new Spark(21);
  Spark l2 = new Spark(25);
  SpeedControllerGroup r = new SpeedControllerGroup(r0, r1, r2);
  SpeedControllerGroup l = new SpeedControllerGroup(l0, l1, l2);
  DifferentialDrive dT = new DifferentialDrive(l, r);
  Joystick driver = new Joystick(0);
  double speed;
  @Override
  public void robotInit() {
    speed = SmartDashboard.getNumber("speed", 0.8);
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
    dT.tankDrive(speed*driver.getRawAxis(1), -speed*driver.getRawAxis(3));
    //dT.arcadeDrive(speed*Math.sqrt(driver.getRawAxis(1)*driver.getRawAxis(1)+driver.getRawAxis(0)*driver.getRawAxis(0)), Math.atan2(driver.getRawAxis(3), driver.getRawAxis(2)));
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
