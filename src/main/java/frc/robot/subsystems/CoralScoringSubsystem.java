// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.KrakenConstants;
import frc.robot.commands.Elevator;

public class CoralScoringSubsystem extends SubsystemBase {

  public SparkMax coralOuttakeWheels = new SparkMax(KrakenConstants.coralOuttakeCanID, MotorType.kBrushless);
  public SparkMax coralOmnis = new SparkMax(KrakenConstants.coralOmnisId, MotorType.kBrushless);

  public DigitalInput coralUltrasonic = new DigitalInput(KrakenConstants.ultrasonicSensor);


 

  /** Creates a new ElevatorSubsystem. */
  public CoralScoringSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
}
}
