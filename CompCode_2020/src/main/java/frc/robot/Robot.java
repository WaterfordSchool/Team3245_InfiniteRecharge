/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;


public class Robot extends TimedRobot {
  
  //Left Drive Motors
  CANSparkMax l1 = new CANSparkMax(RobotMap.L1, MotorType.kBrushless);
  CANSparkMax l2 = new CANSparkMax(RobotMap.L2, MotorType.kBrushless);
  CANSparkMax l3 = new CANSparkMax(RobotMap.L3, MotorType.kBrushless);
    
  //Right Drive Motors
  CANSparkMax r1 = new CANSparkMax(RobotMap.R1, MotorType.kBrushless);
  CANSparkMax r2 = new CANSparkMax(RobotMap.R2, MotorType.kBrushless);
  CANSparkMax r3 = new CANSparkMax(RobotMap.R3, MotorType.kBrushless);

  //Speed Controller Groups
  SpeedControllerGroup r = new SpeedControllerGroup(r1, r2, r3);
  SpeedControllerGroup l = new SpeedControllerGroup(l1, l2, l3);

  //Drive Train
  DifferentialDrive dT = new DifferentialDrive(l, r);

  //Floor Loading Motors
  WPI_TalonSRX arm = new WPI_TalonSRX(RobotMap.ARM_MOTOR_ID);
  WPI_TalonSRX intake = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_ID);
  WPI_TalonSRX uptake = new WPI_TalonSRX(RobotMap.UPTAKE_MOTOR_ID);

  //Feeding Motors
  WPI_TalonSRX flywheel = new WPI_TalonSRX(RobotMap.FLYWHEEL_MOTOR_ID);
  WPI_TalonSRX index = new WPI_TalonSRX(RobotMap.INDEX_MOTOR_ID);
  WPI_TalonSRX agitator = new WPI_TalonSRX(RobotMap.AGIT_MOTOR_ID);

  //Climbing Motors
  WPI_TalonSRX climbLeft = new WPI_TalonSRX(RobotMap.CLIMBER_MOTOR_L);
  WPI_TalonSRX climbRight = new WPI_TalonSRX(RobotMap.CLIMBER_MOTOR_R);
  WPI_TalonSRX climbHook = new WPI_TalonSRX(RobotMap.CLIMBER_MOTOR_M);

  //Joysticks
  Joystick driver = new Joystick(RobotMap.DRIVER_PORT);
  Joystick operator = new Joystick(RobotMap.OPERATOR_PORT);

  //Timer
  Timer timer = new Timer();
  double autoStartTime;
  double timeStart;
  double currentTime;

  //Gyro
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  double i,d = 0;
  double p = Math.pow(0.5, 9);
  PIDController PID = new PIDController(p, i, d);

  //SmartDashboard
  SmartDashboard smrt;
  SendableChooser<Integer> choose = new SendableChooser<>();
  SendableChooser<Integer> side = new SendableChooser<>();

  //Limit Switch
  /**DigitalInput armDownSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH_D_PORT);
  DigitalInput armUpSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH_U_PORT);
  boolean isDown = false;
  boolean isPressed = false;
  */

  @Override
  public void robotInit() {
    r.setInverted(true);
    l.setInverted(true);
    side.setDefaultOption("right", 1);
    side.addOption("left", -1);
    choose.setDefaultOption("Auto: feed", 1);
    choose.addOption("Auto: move", 2);
    smrt.putData("side:", side);
    smrt.putData("Auto type:", choose);
    smrt.putNumber("delay time", 0);
  }

  @Override
  public void autonomousInit() {
    timer.start();
    timer.reset();
  }

  @Override
  public void autonomousPeriodic() {
    //smrt.putString("1 = feeding routine; 2 = drive off line", "");
    //double autoNum = smrt.getNumber("Auto Number: ", 1);
    int autoNuma = (int)choose.getSelected();
    double delay = smrt.getNumber("delay time", 0);
    if(autoNuma == 1){
      auto1(delay);
    }
    else if(autoNuma == 2){
      auto2(delay);
    }
  }

  //Feeding auto routine
  public void auto1(double delay){
    if(timer.get()<0.0+delay){
      index.set(0);
      agitator.set(0);
      flywheel.set(0);
      dT.arcadeDrive(0, 0);
      intake.set(0);
      uptake.set(0);
    }else if(timer.get()<3.0+delay){
      intake.set(0);
      uptake.set(0);
      index.set(RobotMap.INDEX_AGIT_SPEED);
      agitator.set(RobotMap.AGIT_SPEED);
      flywheel.set(RobotMap.FLYWHEEL_SPEED*0.5);
    }else if(timer.get()<3.59375+delay){ //next time either 3.59375 or 3.53125
      dT.arcadeDrive(0, ((double)side.getSelected())*0.5);
      index.set(0);
      agitator.set(0);
      flywheel.set(0);
    }else if(timer.get()<15.0){
      dT.arcadeDrive(0.4, 0);
      intake.set(-RobotMap.INTAKE_UPTAKE_SPEED);
      uptake.set(RobotMap.INTAKE_UPTAKE_SPEED);
      index.set(RobotMap.INDEX_AGIT_SPEED);
      agitator.set(RobotMap.AGIT_SPEED);
      flywheel.set(RobotMap.FLYWHEEL_SPEED*0.5);
    }
    else if(timer.get() > 6.0+delay){
      dT.tankDrive(0, 0);
      intake.set(0);
      uptake.set(0);
      index.set(0);
      agitator.set(0);
      flywheel.set(0);
    }

    if(timer.get()<1.75){
      arm.set(RobotMap.ARM_SPEED);
    }else if (timer.get()>1){
      arm.set(0);
    }
  }

  //Off line auto routine
  public void auto2(double delay){
    if(timer.get()<0.0+delay){
      index.set(0);
      agitator.set(0);
      flywheel.set(0);
      dT.arcadeDrive(0, 0);
      intake.set(0);
      uptake.set(0);
    }else if(timer.get() < 3.0+delay){
      dT.arcadeDrive(0.4, 0);
    }
    else if(timer.get() > 5.0+delay){
      dT.tankDrive(0, 0);
    }
  }

  @Override
  public void teleopInit() {
    l1.setOpenLoopRampRate(RobotMap.RAMP_VAL);
    l2.setOpenLoopRampRate(RobotMap.RAMP_VAL);
    l3.setOpenLoopRampRate(RobotMap.RAMP_VAL);
    r1.setOpenLoopRampRate(RobotMap.RAMP_VAL);
    r2.setOpenLoopRampRate(RobotMap.RAMP_VAL);
    r3.setOpenLoopRampRate(RobotMap.RAMP_VAL);
  }

  @Override
  public void teleopPeriodic() {
    currentTime = Timer.getFPGATimestamp();
    dT.arcadeDrive(driver.getRawAxis(RobotMap.DRIVER_LEFT_AXIS) * RobotMap.DRIVE_SPEED, -driver.getRawAxis(RobotMap.DRIVER_RIGHT_AXIS) * RobotMap.DRIVE_SPEED);
    armUpDown();
    intakeUptake();
    indexAgitator();
    flywheel();
    deployClimber();
    speedButton();
    Agitator();
    arm();
    hook();
  }

  @Override
  public void testInit() {
    gyro.reset();
    timer.start();
    timer.reset();
    while (timer.get()<6){
      gyro.calibrate();
    }
  }

  @Override
  public void testPeriodic() {
    turnTo(90, 0);
  }

  //Arm Method
  public void armUpDown(){
    if(operator.getRawButton(RobotMap.OPERATOR_ARM_DOWN_BUTTON)){
      arm.set(RobotMap.ARM_SPEED);
    }
    if(operator.getRawButton(RobotMap.OPERATOR_ARM_UP_BUTTON)){
      arm.set(-RobotMap.ARM_SPEED);
    }
    if(!operator.getRawButton(RobotMap.OPERATOR_ARM_DOWN_BUTTON) && !operator.getRawButton(RobotMap.OPERATOR_ARM_UP_BUTTON)){
      arm.set(0.0);
    }
  }
  public void arm(){
    arm.set(RobotMap.ARM_SPEED*operator.getRawAxis(RobotMap.OPERATOR_ARM_AXIS));
  }
  //Intake Uptake methods
  public void intakeUptake() {
    if(driver.getRawButton(RobotMap.DRIVER_INTAKE_UPTAKE_BUTTON)){
      intake.set(RobotMap.INTAKE_UPTAKE_SPEED);
      uptake.set(-RobotMap.INTAKE_UPTAKE_SPEED);
    }
    else if(driver.getRawButton(RobotMap.DRIVER_OUTAKE_DWTAKE_BUTTON)){
      intake.set(-RobotMap.INTAKE_UPTAKE_SPEED);
      uptake.set(RobotMap.INTAKE_UPTAKE_SPEED);
    }
    else if (!(driver.getRawButton(RobotMap.DRIVER_INTAKE_UPTAKE_BUTTON)&&driver.getRawButton(RobotMap.DRIVER_OUTAKE_DWTAKE_BUTTON))){
      intake.set(0.0);
      uptake.set(0.0);
    }
  }

  //Feeder Methods
  public void indexAgitator() {
    if(operator.getRawButton(RobotMap.OPERATOR_INDEXER_AGIT_BUTTON)){
      index.set(RobotMap.INDEX_AGIT_SPEED);
      agitator.set(RobotMap.AGIT_SPEED);
    }
    else if (!operator.getRawButton(RobotMap.OPERATOR_INDEXER_AGIT_BUTTON)){
      index.set(0.0);
      agitator.set(0.0);

    }
 }

 //Agitator Only
 public void Agitator (){
  if(operator.getRawButton(RobotMap.OPERATOR_AGIT_BUTTON)){
    agitator.set(RobotMap.AGIT_SPEED);
  }
  else if(!operator.getRawButton(RobotMap.OPERATOR_AGIT_BUTTON)&&!operator.getRawButton(RobotMap.OPERATOR_INDEXER_AGIT_BUTTON)){
    agitator.set(0.0);
  }
 }

 public void flywheel(){
   if(operator.getRawButton(RobotMap.OPERATOR_FLYWHEEL_BUTTON)){
     flywheel.set(RobotMap.FLYWHEEL_SPEED);
   }
   else if(operator.getRawButton(RobotMap.OPERATOR_FLYWHEEL_SLow_BUTTON)){
     flywheel.set(RobotMap.FLYWHEEL_SPEED*0.5);
   }
   else if (!operator.getRawButton(RobotMap.OPERATOR_FLYWHEEL_BUTTON)){
     flywheel.set(0.0);
   }
 }

 //Climber Methods
 public void deployClimber() {
   if(operator.getRawButton(RobotMap.OPERATOR_CLIMBER_UP_BUTTON)){
      climbLeft.set(RobotMap.CLIMB_SPEED);
      climbRight.set(RobotMap.CLIMB_SPEED);
   }
   if(operator.getRawButton(RobotMap.OPERATOR_CLIMBER_DOWN_BUTTON)){
      climbLeft.set(-RobotMap.CLIMB_SPEED);
      climbRight.set(-RobotMap.CLIMB_SPEED);
   }
   if(!operator.getRawButton(RobotMap.OPERATOR_CLIMBER_UP_BUTTON)&&!operator.getRawButton(RobotMap.OPERATOR_CLIMBER_DOWN_BUTTON)){
      climbLeft.set(0);
      climbRight.set(0);
   }
 }
  public void hook(){
      if(Math.abs(operator.getRawAxis(RobotMap.OPERATOR_HOOK_AXIS))>0.5){
        climbHook.set(0.5*Math.pow(operator.getRawAxis(RobotMap.OPERATOR_HOOK_AXIS), 3));
      }else if (Math.abs(operator.getRawAxis(RobotMap.OPERATOR_HOOK_AXIS))<0.5){
        climbHook.set(0);
      }
    
  }

 //Speed Button(s)
 public void speedButton(){
   if(driver.getRawButton(RobotMap.DRIVER_FAST_BUTTON)){
    dT.arcadeDrive(driver.getRawAxis(RobotMap.DRIVER_LEFT_AXIS)*RobotMap.DRIVE_FAST_SPEED, -driver.getRawAxis(RobotMap.DRIVER_RIGHT_AXIS)*RobotMap.DRIVE_FAST_SPEED);
   }else if(driver.getRawButton(RobotMap.DRIVER_SLOW_BUTTON)){
    dT.arcadeDrive(driver.getRawAxis(RobotMap.DRIVER_LEFT_AXIS) * RobotMap.DRIVE_SLOW_SPEED, -driver.getRawAxis(RobotMap.DRIVER_RIGHT_AXIS) * RobotMap.DRIVE_SLOW_SPEED);
   }else if (!driver.getRawButton(RobotMap.DRIVER_SLOW_BUTTON)||!driver.getRawButton(RobotMap.DRIVER_FAST_BUTTON)){
    dT.arcadeDrive(driver.getRawAxis(RobotMap.DRIVER_LEFT_AXIS) * RobotMap.DRIVE_SPEED, -driver.getRawAxis(RobotMap.DRIVER_RIGHT_AXIS) * RobotMap.DRIVE_SPEED);
   }
 }

 //Gyro method
 public void turnTo(double targetAngle, double targetSpeed){
  PID.setSetpoint(targetAngle);
  PID.setTolerance(3, 0.1);
  double turn = PID.calculate(gyro.getAngle());
  if(PID.getPositionError()>3){
    dT.tankDrive(turn, -turn);
  }else{
    dT.arcadeDrive(targetSpeed, turn);
  }
  }
 }
