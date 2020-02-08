/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.TimedRobot;


public class Robot extends TimedRobot {
  
  //Left Drive Motors
  CANSparkMax l1 = new CANSparkMax(20, MotorType.kBrushless);
  CANSparkMax l2 = new CANSparkMax(21, MotorType.kBrushless);
  CANSparkMax l3 = new CANSparkMax(25, MotorType.kBrushless);

  //Right Drive Motors
  CANSparkMax r1 = new CANSparkMax(22, MotorType.kBrushless);
  CANSparkMax r2 = new CANSparkMax(23, MotorType.kBrushless);
  CANSparkMax r3 = new CANSparkMax(24, MotorType.kBrushless);

  //Speed Controller Groups
  SpeedControllerGroup r = new SpeedControllerGroup(r1, r2, r3);
  SpeedControllerGroup l = new SpeedControllerGroup(l1, l2, l3);

  //Drive Train
  DifferentialDrive dT = new DifferentialDrive(l, r);

  //Joysticks
  Joystick driver = new Joystick(0);


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
    dT.tankDrive(driver.getRawAxis(1) * RobotMap.DRIVE_SPEED, driver.getRawAxis(3) * RobotMap.DRIVE_SPEED);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
