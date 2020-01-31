/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  //Motors
  Talon right = new Talon(RobotMap.RIGHT_ID);
  Talon left = new Talon(RobotMap.LEFT_ID);

  Talon flywheelMotor = new Talon(RobotMap.FLYWHEEL_MOTOR_ID);
  Talon indexMotor = new Talon(RobotMap.INDEX_MOTOR_ID);

  DifferentialDrive dT = new DifferentialDrive(left, right);

  //Joystick
  Joystick driver = new Joystick(RobotMap.JOYSTICK_DRIVER_PORT_ID); //driver joystick
  Joystick operator = new Joystick(RobotMap.JOYSTICK_OPERATOR_PORT_ID); //operator joystick
  double leftStickVal; //Assigned in teleopPeriodic
  double rightStickVal; //Assigned in teleopPeriodic

  //Sensors
  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
  ADXRS450_Gyro gyro = new ADXRS450_Gyro(kGyroPort); //check SPI, default is CS0, SPI.Port.kMXP

  @Override
  public void robotInit() {

    gyro.calibrate(); //sets gyro's current postition to 0

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    
    //gyro.calibrate();

  }

  @Override
  public void autonomousPeriodic() {
    autoRoutine();
    //SmartDashboard.putNumber("Gyro Value: ", gyro.getAngle());

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
    if (operator.getRawButton(RobotMap.FLYWHEEL_SLOW_BUTTON_ID)){
      flywheelMotor.set(RobotMap.FLYWHEEL_SLOW_SPEED_ID);
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

    /*Sets Gyro Turning Value 
    *double turnVal = (RobotMap.GYRO_SETPOINT - gyro.getAngle()) * RobotMap.GYRO_TURNING_CONSTANT;
    *turnVal = Math.copySign(turnVal, leftStickVal);
    *turnVal = Math.copySign(turnVal, rightStickVal);
    */

    //Tank drive method call
    dT.tankDrive(-leftStickVal * RobotMap.DRIVE_SPEED_ID, -rightStickVal * RobotMap.DRIVE_SPEED_ID);
    //dT.arcadeDrive(driver.getY(), turningVal);

    index();
    flyWheel();

    //SmartDashboard code
    //SmartDashboard.putNumber("Gyro Value: ", gyro.getAngle());
  }

  //Test AutoRoutine
  public void autoRoutine() {
    //resets gyro to 0
    //double turningVal = (RobotMap.GYRO_SETPOINT - gyro.getAngle()) * RobotMap.GYRO_TURNING_CONSTANT;
   // double startAngle = gyro.getAngle(); //should be 0 intially

    //turn right 45 degrees
    //left off here decreasing turning value as the gyro gets closer to 45 degrees
    /*if (gyro.getAngle() < 45){
      //right.set(Math.pow(45 - startAngle / -45, 3));
      //left.set(Math.pow(45 - startAngle / 45, 3));
      right.set(-0.3);
      left.set(0.3);
    }
    else{
      right.set(0.0);
      left.set(0.0);
    }
    */
    SmartDashboard.putNumber("Gyro Value: ", gyro.getAngle());
  }

  @Override
  public void testPeriodic() {
  }

}
