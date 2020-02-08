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
  Joystick driver = new Joystick(RobotMap.DRIVER_PORT);
  Joystick operator = new Joystick(RobotMap.OPERATOR_PORT);

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
    dT.tankDrive(driver.getRawAxis(RobotMap.DRIVER_LEFT_AXIS) * RobotMap.DRIVE_SPEED, driver.getRawAxis(RobotMap.DRIVER_RIGHT_AXIS) * RobotMap.DRIVE_SPEED);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  /*Auxilary Methods
    *Floor Loading Methods
      *arm down
      *run intake/run uptake
    *Feeder Methods
      *run indexer & agitator
      *run flywheel
    *Climber Methods
      *deploy tubes
      *retract tubes (maybe)
    *Slow Button
  */
}
