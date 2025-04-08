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
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.KrakenConstants;
import frc.robot.commands.Elevator;

public class Elastic extends SubsystemBase {

  public final BooleanSubscriber bottomBoolSub;
  public final BooleanPublisher bottomBoolPub;
  public final BooleanSubscriber topBoolSub;
  public final BooleanPublisher topBoolPub;

  public final DoubleSubscriber encoderSub;
  public final DoublePublisher encoderPub;

  public final BooleanSubscriber RobotOrientedSub;
  public final BooleanPublisher RobotOrientedPub;

  public final DoubleSubscriber AprilTxSub;
  public final DoublePublisher AprilTxPub;
  public final DoubleSubscriber AprilTySub;
  public final DoublePublisher AprilTyPub;
  public final DoubleSubscriber AprilTaSub;
  public final DoublePublisher AprilTaPub;
  public final BooleanSubscriber AprilTvSub;
  public final BooleanPublisher AprilTvPub;

  public final BooleanSubscriber coralSensorSub;
  public final BooleanPublisher coralSensorPub;

  public final DoubleSubscriber gyroSub;
  public final DoublePublisher gyroPub;

  public final DoubleSubscriber frontLeftDistanceSub;
  public final DoublePublisher frontLeftDistancePub;

  public final BooleanSubscriber algaeOutSub;
  public final BooleanPublisher algaeOutPub;
  public final BooleanSubscriber algaeInSub;
  public final BooleanPublisher algaeInPub;

  // public final DoubleSubscriber algaeEncoderSub;
  // public final DoublePublisher algaeEncoderPub;

  /** Creates a new ElevatorSubsystem. */
  public Elastic() {
     NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable datatable = inst.getTable("Shuffleboard");

    bottomBoolSub = datatable.getBooleanTopic("Bottom Limit Switch").subscribe(false);
    bottomBoolPub = datatable.getBooleanTopic("Bottom Limit Switch").publish();

    topBoolSub = datatable.getBooleanTopic("Top Limit Switch").subscribe(false);
    topBoolPub = datatable.getBooleanTopic("Top Limit Switch").publish();

    encoderSub = datatable.getDoubleTopic("Kraken Encoder").subscribe(0);
    encoderPub = datatable.getDoubleTopic("Kraken Encoder").publish();

    RobotOrientedSub = datatable.getBooleanTopic("FieldOriented").subscribe(false);
    RobotOrientedPub = datatable.getBooleanTopic("FieldOriented").publish();

    AprilTxSub = datatable.getDoubleTopic("Tx").subscribe(0);
    AprilTxPub = datatable.getDoubleTopic("Tx").publish();
    AprilTySub = datatable.getDoubleTopic("Ty").subscribe(0);
    AprilTyPub = datatable.getDoubleTopic("Ty").publish();
    AprilTaSub = datatable.getDoubleTopic("Ta").subscribe(0);
    AprilTaPub = datatable.getDoubleTopic("Ta").publish();
    AprilTvSub = datatable.getBooleanTopic("Tv").subscribe(false);
    AprilTvPub = datatable.getBooleanTopic("Tv").publish();

    coralSensorSub = datatable.getBooleanTopic("Coral Loaded?").subscribe(false);
    coralSensorPub = datatable.getBooleanTopic("Coral Loaded?").publish();

    gyroSub = datatable.getDoubleTopic("Gyro").subscribe(0);
    gyroPub = datatable.getDoubleTopic("Gyro").publish();

    frontLeftDistanceSub = datatable.getDoubleTopic("Front Left Distance").subscribe(0);
    frontLeftDistancePub = datatable.getDoubleTopic("Front Left Distance").publish();

    algaeOutSub = datatable.getBooleanTopic("Algae Out Limit Switch").subscribe(false);
    algaeOutPub = datatable.getBooleanTopic("Algae Out Limit Switch").publish();

    algaeInSub = datatable.getBooleanTopic("Algae In Limit Switch").subscribe(false);
    algaeInPub = datatable.getBooleanTopic("Algae In Limit Switch").publish();

    // algaeEncoderSub = datatable.getDoubleTopic("Algae Encoder").subscribe(0);
    // algaeEncoderPub = datatable.getDoubleTopic("Algae Encoder").publish();
  }
  

  @Override
  public void periodic() {
}
}

