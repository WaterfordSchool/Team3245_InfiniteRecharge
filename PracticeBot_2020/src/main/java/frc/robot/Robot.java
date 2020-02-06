/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  Timer counter = new Timer();
  
  ADXRS450_Gyro gyro = new ADXRS450_Gyro(); //check SPI, default is CS0, SPI.Port.kMXP
  //private static final double kVoltsPerDegreePerSecond = 0.0120;
  //private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
  private static final double kAngleSetpoint = 0.0;
  private static final double kP = 1.0/180.0; // propotional turning constant

  PIDController pidLoop = new PIDController(1, 0, 0);

  @Override
  public void robotInit() {
    gyro.calibrate();  //calibrates gyro on robot start
    gyro.reset();  //sets gyro's current postition to 0
  }

  @Override
  public void robotPeriodic() {
    //autoRoutine();
    //gyroValue+= Math.ceil(gyro.getAngle()*100-2)/100;
    //gyroRateRate= Math.abs(gyroRate - gyro.getRate());        
    //SmartDashboard.putNumber("heading", gyro.getAngle());
    //SmartDashboard.putNumber("gyro rate change", gyroRateRate);
    //gyroRate = gyro.getRate();
  }

  @Override
  public void autonomousInit() {
    counter.reset();
    counter.start();
    gyro.reset();

  }

  @Override
  public void autonomousPeriodic() {
    autoRoutine();

  }

  public void index(){
    if(operator.getRawButton(RobotMap.INDEX_BUTTON_ID)){
      indexMotor.set(RobotMap.INDEX_SPEED_ID);
    }
    else{
      indexMotor.set(0.0);
    }
  }

  public void turnTo(double angel, double speedForward){
    dT.arcadeDrive(speedForward, (angel-gyro.getAngle())/180);
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

    //Tank drive method call
    dT.tankDrive(-leftStickVal * RobotMap.DRIVE_SPEED_ID, -rightStickVal * RobotMap.DRIVE_SPEED_ID);
    //dT.arcadeDrive(driver.getY(), turningVal);

    //index and flyWheel methods call
    index();
    flyWheel();
  }

  //Test AutoRoutine
  public void autoRoutine() {
    counter.reset();
    while(counter.get()<3.0){
      indexMotor.set(RobotMap.INDEX_SPEED_ID);
      flywheelMotor.set(RobotMap.FLYWHEEL_SLOW_SPEED_ID);
      right.set(0.0);
      left.set(0.0);
    }

    counter.reset();
    while(counter.get()<4.0){
      indexMotor.set(0.0);
      flywheelMotor.set(0.0);
      //turn right 45 degrees
      if (gyro.getAngle() < 90){
        //right.set(Math.pow(45 - startAngle / -45, 3));
        //left.set(Math.pow(45 - startAngle / 45, 3));
        right.set(0.3);
        left.set(0.3);
      }
      //stop drive motors
      else{
        right.set(0.0);
        left.set(0.0);
      }
    }

    gyro.reset();
    counter.reset();
    double initialValue = gyro.getAngle()/180.0;
    while (counter.get()<20){
      double turningValue = initialValue-(kAngleSetpoint - gyro.getAngle()) * kP;
      // Invert the direction of the turn if we are going backwards
      turningValue = Math.copySign(turningValue, 1);
      dT.arcadeDrive(0.4, turningValue);
      SmartDashboard.putNumber("heading", gyro.getAngle());
      SmartDashboard.putNumber("turning value", -turningValue);
    }
    
    counter.reset();
    while (counter.get()<1.0){
      indexMotor.set(0.0);
      flywheelMotor.set(0.0);
      right.set(0.0);
      left.set(0.0);
    }
    
    
    //left off here decreasing turning value as the gyro gets closer to 45 degrees
    //double turningVal = (RobotMap.GYRO_SETPOINT - gyro.getAngle()) * RobotMap.GYRO_TURNING_CONSTANT;
    //double startAngle = gyro.getAngle(); //should be 0 intially
    
    //Print values to SmartDashboard
    SmartDashboard.putNumber("Gyro Value: ", gyro.getAngle());
    SmartDashboard.putNumber("Counter: ", counter.get());
    //SmartDashboard.putNumber("gyro rate", gyro.getRate());

    //Print values to RioLog
    //System.out.println(gyro.getAngle());
    //System.out.println("rate"+gyro.getRate());
  }

  @Override
  public void testPeriodic() {
  }

}
