// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Limelight Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.util.ElasticSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;



public class LimelightSubsystem extends SubsystemBase {

  LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ledMode = table.getEntry("ledMode");
  public NetworkTableEntry tv = table.getEntry("tv");

    public double x;
    public double y;
    public double area;
    public boolean validID;

   HttpCamera limelightFeed;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    limelightFeed = new HttpCamera("Limelight Feed", "http://172.28.0.1:5800", HttpCameraKind.kMJPGStreamer);
    CameraServer.startAutomaticCapture(limelightFeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    validID = (tv.getDouble(0) == 1);

    RobotContainer.m_Elastic.AprilTxPub.set(x);
    RobotContainer.m_Elastic.AprilTyPub.set(y);
    RobotContainer.m_Elastic.AprilTaPub.set(area);
    RobotContainer.m_Elastic.AprilTvPub.set(validID);

    // CameraServer.getVideo(limelightFeed); 
    
  
   
  }

  /* method that takes in tx, ty, and ta, as well as desired tx, ty, and ta
   * 
   * Create a trajectory based on this information
   * 
   * Run trajectory
   */

   public void RangeAim(double x, double y, double range) {

    double ySpeed;
    double xSpeed;

    ySpeed = limelightYSpeed(y, range) / 25;
    xSpeed = limelightXSpeed(x, range) / 25;
    RobotContainer.m_robotDrive.drive(xSpeed, ySpeed, 0, false);
   }

   public void RangeAimTa(double x, double a, double range) {
    double ySpeed;
    double xSpeed;

    xSpeed = limelightXSpeedTa(a, range) / 25;
    ySpeed = limelightYSpeed(x, range) / 25;
    RobotContainer.m_robotDrive.drive(xSpeed, ySpeed, 0, false);
   }

   public void RangeAimTaAuton(double x, double a, double range) {
    double ySpeed;
    double xSpeed;

    xSpeed = limelightXSpeedTa(a, range) / 50;
    ySpeed = limelightYSpeed(x, range) / 50;
    RobotContainer.m_robotDrive.drive(xSpeed, ySpeed, 0, false);
   }

   public double limelightXSpeedTa(double taGoal, double range) {
    double ySpeed;

    if (Math.abs(taGoal - ta.getDouble(0)) > range) {
      ySpeed = 0.5*(taGoal - ta.getDouble(0));
    }
    else {
      ySpeed = 0;
    }
    return ySpeed;
   }

   

   public double limelightYSpeed(double txGoal, double range) {
    double ySpeed;
    
    if (Math.abs(txGoal - tx.getDouble(0.0)) > range) {
     ySpeed = 0.5*(txGoal - tx.getDouble(0.0));
    }
    else {
      ySpeed = 0;
    }
    return ySpeed;
   }

   public double limelightXSpeed(double tyGoal, double range) {
    double xSpeed;

    if (Math.abs(tyGoal - ty.getDouble(0.0)) > range) {
      xSpeed = 0.5*(tyGoal - ty.getDouble(0.0));
   }
   else {
      xSpeed = 0;
   }
    return xSpeed;
   }



   // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.

  /* 
  double limelight_aim_proportional(double aimCenter)
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = (LimelightHelpers.getTX("limelight") - aimCenter) * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }
  */

  /* 
  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional(double rangeCenter)
  {    
    double kP = .1;
    double targetingForwardSpeed = (LimelightHelpers.getTY("limelight") - rangeCenter) * kP;
    targetingForwardSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }
  */


}
