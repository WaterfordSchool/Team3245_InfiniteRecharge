/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  //all right motors have even ID's, all left have odd
  public Spark r1 = new Spark(0);                                     //right motor with CAN ID 0
  public Spark r2 = new Spark(2);                                     //right motor with CAN ID 2
  public Spark r3 = new Spark(4);                                     //right motor with CAN ID 4
  public Spark l1 = new Spark(1);                                     //left  motor with CAN ID 1
  public Spark l2 = new Spark(3);                                     //left  motor with CAN ID 3
  public Spark l3 = new Spark(5);                                     //left  motor with CAN ID 5
  public SpeedControllerGroup r = new SpeedControllerGroup(r1, r2, r3); //groups all the right motors together so they share the same action
  public SpeedControllerGroup l = new SpeedControllerGroup(l1, l2, l3); //groups all the left  motors together so they share the same action
  public Joystick d = new Joystick(0);                                //the driver controller
  double speed = 0;                                                   //the speed variable
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain(double speed) {
    this.speed = speed;
  }
  /**
   * The method to controll the drivebase is written here, actually controlling it should be written in the Command.
   * @param left  the fraction of the max speed that the left  side drives at
   * @param right the fraction of the max sleed that the right side drives at
   */
  public void drive(double left, double right){
    l.set(left *speed);
    r.set(right*speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
