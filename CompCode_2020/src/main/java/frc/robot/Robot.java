/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

  //Floor Loading Motors
  WPI_TalonSRX arm = new WPI_TalonSRX(RobotMap.ARM_MOTOR_ID);
  WPI_TalonSRX intake = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_ID);
  WPI_TalonSRX uptake = new WPI_TalonSRX(RobotMap.UPTAKE_MOTOR_ID);

  //Feeding Motors
  WPI_TalonSRX flywheel = new WPI_TalonSRX(RobotMap.FLYWHEEL_MOTOR_ID);
  WPI_TalonSRX indexAgitator = new WPI_TalonSRX(RobotMap.INDEX_AGIT_MOTOR_ID);

  //Climbing Motors
  WPI_TalonSRX climbLeft = new WPI_TalonSRX(RobotMap.CLIMBER_MOTOR_L);
  WPI_TalonSRX climbRight = new WPI_TalonSRX(RobotMap.CLIMBER_MOTOR_R);

  //Joysticks
  Joystick driver = new Joystick(RobotMap.DRIVER_PORT);
  Joystick operator = new Joystick(RobotMap.OPERATOR_PORT);

  //Timer
  Timer timer = new Timer();
  double autoStartTime;
  double timeElapsed;
  double currentTime;

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
    currentTime = Timer.getFPGATimestamp();
    dT.tankDrive(driver.getRawAxis(RobotMap.DRIVER_LEFT_AXIS) * RobotMap.DRIVE_SPEED, driver.getRawAxis(RobotMap.DRIVER_RIGHT_AXIS) * RobotMap.DRIVE_SPEED);
    armDown();
    intakeUptake();
    indexAgitator();
    flywheel();
    deployClimber();
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

  //Floor Loading Methods
  public void armDown(){
    timeElapsed = currentTime - autoStartTime;
    boolean isDown = true;
    if(driver.getRawButton(RobotMap.DRIVER_ARM_BUTTON)){
      while(timeElapsed < 3){
        if(isDown){
          arm.set(RobotMap.ARM_SPEED);
        }
        if(!isDown){
          arm.set(-RobotMap.ARM_SPEED);
        }
      }
    }
    isDown = !isDown;
  }

  public void intakeUptake() {
    if(driver.getRawButton(RobotMap.DRIVER_INTAKE_UPTAKE_BUTTON)){
      intake.set(RobotMap.INTAKE_UPTAKE_SPEED);
      uptake.set(RobotMap.INTAKE_UPTAKE_SPEED);
    }
    else{
      intake.set(0.0);
      uptake.set(0.0);
    }
  }

  //Feeder Methods
  public void indexAgitator() {
    if(driver.getRawButton(RobotMap.OPERATOR_INDEXER_AGIT_BUTTON)){
      indexAgitator.set(RobotMap.INDEX_AGIT_SPEED);
    }
    else{
      indexAgitator.set(0.0);
    }
 }

 public void flywheel(){
   if(driver.getRawButton(RobotMap.OPERATOR_FLYWHEEL_BUTTON)){
     flywheel.set(RobotMap.FLYWHEEL_SPEED);
   }
   else{
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
 }
}
