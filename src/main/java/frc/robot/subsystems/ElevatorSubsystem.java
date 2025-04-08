// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import frc.robot.Constants.KrakenConstants;
import frc.robot.commands.Elevator;

public class ElevatorSubsystem extends SubsystemBase {

  public TalonFX elevatorLift = new TalonFX(KrakenConstants.krakenPortID);

  public DigitalInput bottomLimitSwitch = new DigitalInput(KrakenConstants.bottomLimitSwitchPort);
  public DigitalInput topLimitSwitch = new DigitalInput(KrakenConstants.topLimitSwitchport);

  boolean topLimitSwitchValue;
  boolean bottomLimitSwitchValue;
  double encoderCounts;

  double feedforward = -0.027222;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // in init function
      var talonFXConfigs = new TalonFXConfiguration();


      // set slot 0 gains
      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
      slot0Configs.kV = 1.5; // A velocity target of 1 rps results in 0.12 V output
      slot0Configs.kA = 0.5; // An acceleration of 1 rps/s requires 0.01 V output
      slot0Configs.kP = 0.5; // A position error of 2.5 rotations results in 12 V output
      slot0Configs.kI = 0.15; // no output for integrated error 0.05
      slot0Configs.kD = 0.05; // A velocity error of 1 rps results in 0.1 V output
      slot0Configs.kG = -0.031;


      // set Motion Magic settings
      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 4; // Target cruise velocity of 80 rps
      motionMagicConfigs.MotionMagicAcceleration = 2; // Target acceleration of 160 rps/s (0.5 seconds)
      motionMagicConfigs.MotionMagicJerk = 0.5; // Target jerk of 1600 rps/s/s (0.1 seconds)

      elevatorLift.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
  }

  public void elevatorAuton(double target) {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    elevatorLift.setControl(m_request.withPosition(-target));
  }
}
