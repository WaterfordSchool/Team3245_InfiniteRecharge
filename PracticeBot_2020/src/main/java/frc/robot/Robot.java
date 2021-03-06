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
  private static double kP = 1.0; // propotional turning constant
  double turn;
  double p = 0.015625;
  double i = 0/*.015625*/;
  double d = 0;
  double t = 0.05;
  PIDController pidLoop = new PIDController(p, i, d);
  double jyro=0;

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
    SmartDashboard.putNumber("heading", gyro.getAngle());
    SmartDashboard.putNumber("gyro Velocity", jyro-gyro.getAngle());
    //SmartDashboard.putNumber("gyro rate change", gyroRateRate);
    //gyroRate = gyro.getRate();
    jyro = gyro.getAngle();

  }

  @Override
  public void autonomousInit() {
    counter.reset();
    counter.start();
  }
  int n = 0;
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Counter: ", counter.get());
    if (counter.get()<3){
      indexMotor.set(RobotMap.INDEX_SPEED_ID);
      flywheelMotor.set(RobotMap.FLYWHEEL_SLOW_SPEED_ID);
      right.set(0.0);
      left.set(0.0);
      gyro.reset();
    }else if (counter.get()<6){
      turnTo(90, 0.2);
    }else if (counter.get()<9){
      turnTo(90, 0.8);
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
  public void turnTo(final double angel, final double speedForward) {
    pidLoop.setSetpoint(angel);
    pidLoop.setTolerance(3, 0.1);
    double turn = pidLoop.calculate(gyro.getAngle());
    dT.arcadeDrive(speedForward, turn);
    
  }

  public void flyWheel() {
    if (operator.getRawButton(RobotMap.FLYWHEEL_BUTTON_ID)) {
      flywheelMotor.set(RobotMap.FLYWHEEL_SPEED_ID);
    }
    if (operator.getRawButton(RobotMap.FLYWHEEL_SLOW_BUTTON_ID)) {
      flywheelMotor.set(RobotMap.FLYWHEEL_SLOW_SPEED_ID);
    } else {
      flywheelMotor.set(0.0);
    }
  }

  @Override
  public void teleopPeriodic() {
    // Assigns values because it needs to be routinely updated
    leftStickVal = driver.getRawAxis(RobotMap.LEFT_AXIS_ID);
    rightStickVal = driver.getRawAxis(RobotMap.RIGHT_AXIS_ID);

    // Tank drive method call
    //if (leftStickVal<0){rightStickVal= -rightStickVal;}
    dT.arcadeDrive(leftStickVal * RobotMap.DRIVE_SPEED_ID, driver.getRawAxis(2) * RobotMap.DRIVE_SPEED_ID);
    //dT.arcadeDrive(driver.getY(), turningVal);
    //dT.tankDrive(rightStickVal * RobotMap.DRIVE_SPEED_ID, leftStickVal * RobotMap.DRIVE_SPEED_ID); //This is backwards rn, normally its -leftStick, -rightStick I just needed to flip it because the ramp was backwards and its weird to drive like that lol

    // index and flyWheel methods call
    index();
    flyWheel();
  }

  // Test AutoRoutine
  public void autoRoutine() {
    counter.reset();
    while (counter.get() < 3.0) {
      indexMotor.set(RobotMap.INDEX_SPEED_ID);
      flywheelMotor.set(RobotMap.FLYWHEEL_SLOW_SPEED_ID);
      right.set(0.0);
      left.set(0.0);
      SmartDashboard.putNumber("Counter: ", counter.get());
    }

    counter.reset();
    gyro.reset();
    while (counter.get() < 4.0) {
      indexMotor.set(0.0);
      flywheelMotor.set(0.0);
      SmartDashboard.putNumber("Counter: ", counter.get());
      // turn right 45 degrees
      turnTo(90, 0);
      if (gyro.getAngle() < 90) {
        // right.set(Math.pow(45 - startAngle / -45, 3));
        // left.set(Math.pow(45 - startAngle / 45, 3));
        right.set(0.3);
        left.set(0.3);
      }
      // stop drive motors
      else {
        right.set(0.0);
        left.set(0.0);
      }
    }
    counter.reset();
    gyro.reset();
    while (counter.get() < 5){
      
    }

    gyro.reset();
    counter.reset();
    final double initialValue = gyro.getAngle();
    while (counter.get()<0){
      SmartDashboard.putNumber("Counter: ", counter.get());
      double turningValue = initialValue-(kAngleSetpoint - gyro.getAngle());
      // Invert the direction of the turn if we are going backwards
      turningValue = Math.copySign(turningValue, 1);
      turningValue *= kP;
      dT.arcadeDrive(0.4, turningValue);
      SmartDashboard.putNumber("turning value", -turningValue);
    }
    
    counter.reset();
    while (counter.get()<1.0){
      SmartDashboard.putNumber("Counter: ", counter.get());
      indexMotor.set(0.0);
      flywheelMotor.set(0.0);
      right.set(0.0);
      left.set(0.0);
    }
    
    
    //left off here decreasing turning value as the gyro gets closer to 45 degrees
    //double turningVal = (RobotMap.GYRO_SETPOINT - gyro.getAngle()) * RobotMap.GYRO_TURNING_CONSTANT;
    //double startAngle = gyro.getAngle(); //should be 0 intially
    
    //Print values to SmartDashboard

    //SmartDashboard.putNumber("gyro rate", gyro.getRate());

    //Print values to RioLog
    //System.out.println(gyro.getAngle());
    //System.out.println("rate"+gyro.getRate());
  }
  double testSped ;
  double testAngel;
  @Override
  public void testPeriodic() {
    turnTo(testAngel, testSped);
  }

  public void testInit(){
    counter.reset();
    gyro.reset();
    p =          SmartDashboard.getNumber("Parallel"   , p);
    i =          SmartDashboard.getNumber("Integral"   , i);
    d =          SmartDashboard.getNumber("Derivative" , d);
    pidLoop.setP(SmartDashboard.getNumber("Parallel"   , p));
    pidLoop.setI(SmartDashboard.getNumber("Integral"   , i));
    pidLoop.setD(SmartDashboard.getNumber("Derivative" , d));

    testSped =   SmartDashboard.getNumber("test speed", 0.5);
    testAngel=   SmartDashboard.getNumber("test angle (degrees)", 90);
    counter.reset();
    counter.start();
  }
  public void gucciGamerMode(){
    turnTo(0, 0.5);
  }
}
