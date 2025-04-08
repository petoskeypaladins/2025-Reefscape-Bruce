// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Configs;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.KrakenConstants;



public class FloorAlgaeSubsystem extends SubsystemBase {
  /** Creates a new FloorAlgae. */
  public FloorAlgaeSubsystem() {}
    public SparkMax algaeFloorPivot = new SparkMax(DriveConstants.floorAlgaePivotCanID, MotorType.kBrushless);
    public SparkMax floorAlgaeSpinner = new SparkMax(DriveConstants.FloorAlgaeSpinnerCanID, MotorType.kBrushless);
    public SparkMax algaeRemoval = new SparkMax(DriveConstants.algaeRemovalCanID, MotorType.kBrushed);
    public SparkMax algaeRemovalSpinner = new SparkMax(DriveConstants.algaeRemovalSpinnerCanID, MotorType.kBrushless);

    public DigitalInput algaeOutLimitSwitch = new DigitalInput(KrakenConstants.algaeOutLimitSwitchPort);
    public DigitalInput algaeInLimitSwitch = new DigitalInput(KrakenConstants.algaeInLimitSwitchPort);
   //public SparkMax coralSpinners = new SparkMax(DriveConstants.coralSpinnerCanID, MotorType.kBrushless);

  
  @Override
  public void periodic() {
    //This method will be called once per scheduler run

    RobotContainer.m_Elastic.algaeOutPub.set(algaeOutLimitSwitch.get());
    RobotContainer.m_Elastic.algaeInPub.set(algaeInLimitSwitch.get());
    // RobotContainer.m_Elastic.algaeEncoderPub.set(algaeRemoval.getEncoder().getPosition());
    
  }

  public static void exampleMethod() {
    
  }
} 