/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {

  //Networking Variables
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;

  //Tuning Porportion
  double speed = 0.8;
  
  //Basic Drivetrain Code
  Joystick driver = new Joystick(0);
  WPI_TalonFX left = new WPI_TalonFX(1);
  WPI_TalonFX right = new WPI_TalonFX(0);
  DifferentialDrive driveTrain = new DifferentialDrive(left, right);
  
  
  @Override
  public void robotInit() {
  }
  
  @Override
  public void robotPeriodic() {
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
    driveTrain.tankDrive(driver.getRawAxis(1) * speed, driver.getRawAxis(3) * speed);
    //driveTrain.arcadeDrive(driver.getRawAxis(1) * speed, driver.getRawAxis(2) * speed);
    Update_Limelight_Tracking();
    boolean auto = driver.getRawButton(2);
      if (auto)
      {
        if (m_LimelightHasValidTarget)
          {
                driveTrain.arcadeDrive(m_LimelightDriveCommand,m_LimelightSteerCommand);
          }
          else
          {
                driveTrain.arcadeDrive(0.0,0.0);
          }
        }
        else
        {
          //driveTrain.tankDrive(driver.getRawAxis(1)*speed,driver.getRawAxis(3));
          driveTrain.arcadeDrive(driver.getRawAxis(1) * speed, -driver.getRawAxis(2) * speed);
        }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.02;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 25.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.5;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * -STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * -DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  }
}
