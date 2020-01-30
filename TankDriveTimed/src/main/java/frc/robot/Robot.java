/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.*;

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

  //Left
  CANSparkMax l1 = new CANSparkMax(20, MotorType.kBrushless);
  CANSparkMax l2 = new CANSparkMax(21, MotorType.kBrushless);
  CANSparkMax l3 = new CANSparkMax(25, MotorType.kBrushless);

  //Right
  CANSparkMax r1 = new CANSparkMax(22, MotorType.kBrushless);
  CANSparkMax r2 = new CANSparkMax(23, MotorType.kBrushless);
  CANSparkMax r3 = new CANSparkMax(24, MotorType.kBrushless);

  SpeedControllerGroup r = new SpeedControllerGroup(r1, r2, r3);
  SpeedControllerGroup l = new SpeedControllerGroup(l1, l2, l3);
  DifferentialDrive dT = new DifferentialDrive(l, r);

  //Joystick
  Joystick driver = new Joystick(0);
  double speed;
  //Gyro
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  @Override
  public void robotInit() {
    speed = SmartDashboard.getNumber("speed", 0.8);
    gyro.calibrate();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    autoRoutine();
    SmartDashboard.putNumber("Gyro Value: ", gyro.getAngle());
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    //Drive Code
    dT.tankDrive(-speed*driver.getRawAxis(1), -speed*driver.getRawAxis(3));
    //dT.arcadeDrive(speed*Math.sqrt(driver.getRawAxis(1)*driver.getRawAxis(1)+driver.getRawAxis(0)*driver.getRawAxis(0)), Math.atan2(driver.getRawAxis(3), driver.getRawAxis(2)));
    
    //SmartDashboard
    SmartDashboard.putNumber("Gyro Value: ", gyro.getAngle()); //just to test if we can recieve values from it
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public void autoRoutine(){
    gyro.calibrate();
    double startAngle = gyro.getAngle();

    while (gyro.getAngle() < 45){
      r.set(Math.pow(45 - startAngle / 45, 3));
      l.set(Math.pow(45 - startAngle / 45, 3));
    }
  }

}
