// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;
import java.rmi.server.RemoteObjectInvocationHandler;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;


/* You should consider using the more terse Command factories API instead 
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevator extends Command {


    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    boolean L1Button = false;
    boolean L2Button = false;
    boolean L3Button = false;
    boolean L4Button = false;

    double L1Target = 0;
    double L2Target = 10.275; //10.5
    double L3Target = 27.5;
    double L4Target = 59; 

    double algae1Removal = 22.5;
    double algae2Removal = 37.7;

/* 
    double kP = 0.45;
    double kI = 0.01;
    double kD = 0;
    PIDController pid = new PIDController(kP/8, kI, kD);
*/
    double target;





  /** Creates a new Elevator. */
  public Elevator() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ElevatorSubsystem);


     // in init function, set slot 0 gains
   /*  var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.25; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    */
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.m_FloorAlgaeSubsystem.algaeInLimitSwitch.get() == false)
      target = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  

 
//Manual Control for Elevator

if (RobotContainer.m_Joystick.getRawButton(2)) {
  
// If moving down and bottom limit switch is pressed, DONT MOVE
  if (RobotContainer.m_Joystick.getRawAxis(1) > 0 && RobotContainer.m_ElevatorSubsystem.bottomLimitSwitch.get() == false) {
    RobotContainer.m_ElevatorSubsystem.elevatorLift.set(0);
  }
// If moving up and bottom limit switch is pressed, MOVE
  else if (RobotContainer.m_Joystick.getRawAxis(1) < 0 && RobotContainer.m_ElevatorSubsystem.bottomLimitSwitch.get() == false) {
    RobotContainer.m_ElevatorSubsystem.elevatorLift.set(-0.25*(RobotContainer.m_CommandJoystick.getRawAxis(1)));
  }
// If moving up and top limit switch is pressed, DONT MOVE
  else if (RobotContainer.m_Joystick.getRawAxis(1) < 0 && RobotContainer.m_ElevatorSubsystem.topLimitSwitch.get() == false) {
    RobotContainer.m_ElevatorSubsystem.elevatorLift.set(0);
  }
// If moving down and top limit switch is pressed, MOVE 
  else if (RobotContainer.m_Joystick.getRawAxis(1) > 0 && RobotContainer.m_ElevatorSubsystem.topLimitSwitch.get() == false) {
    RobotContainer.m_ElevatorSubsystem.elevatorLift.set(-0.25*(RobotContainer.m_CommandJoystick.getRawAxis(1)));
  }
// If no limit switches are pressed, move elevator
  else if (RobotContainer.m_ElevatorSubsystem.bottomLimitSwitch.get() && RobotContainer.m_ElevatorSubsystem.topLimitSwitch.get()) {
    RobotContainer.m_ElevatorSubsystem.elevatorLift.set(-0.25*(RobotContainer.m_CommandJoystick.getRawAxis(1)));
  }
  target = RobotContainer.m_ElevatorSubsystem.elevatorLift.getPosition().getValueAsDouble() - 1;
}


else {


// PID Targets

if (RobotContainer.m_Joystick.getRawButton(5)) {
  if (RobotContainer.m_FloorAlgaeSubsystem.algaeInLimitSwitch.get() == false)
  target = -L1Target;
}

if (RobotContainer.m_Joystick.getRawButton(3)) {
  target = -L2Target;
}

if (RobotContainer.m_Joystick.getRawButton(4)) {
  target = -L3Target;
}

if (RobotContainer.m_Joystick.getRawButton(6)) {
  target = -L4Target;
}

if (RobotContainer.m_Joystick.getRawButton(10)) {
  target = -algae1Removal;
}

if (RobotContainer.m_Joystick.getRawButton(7)) {
  target = -algae2Removal;
}


// Set motor to target
// RobotContainer.m_ElevatorSubsystem.elevatorLift.set(pid.calculate(RobotContainer.m_ElevatorSubsystem.elevatorLift.getPosition().getValueAsDouble(), target));
RobotContainer.m_ElevatorSubsystem.elevatorLift.setControl(m_request.withPosition(target));



}



// This method will be called once per scheduler run

RobotContainer.m_Elastic.bottomBoolPub.set(RobotContainer.m_ElevatorSubsystem.bottomLimitSwitch.get());
RobotContainer.m_Elastic.topBoolPub.set(RobotContainer.m_ElevatorSubsystem.topLimitSwitch.get());
RobotContainer.m_Elastic.encoderPub.set(RobotContainer.m_ElevatorSubsystem.elevatorLift.getPosition().getValueAsDouble());
RobotContainer.m_Elastic.coralSensorPub.set(RobotContainer.m_CoralScoringSubsystem.coralUltrasonic.get());


if (RobotContainer.m_ElevatorSubsystem.bottomLimitSwitch.get() == false) {
RobotContainer.m_ElevatorSubsystem.elevatorLift.setPosition(0);
}

}

  



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_ElevatorSubsystem.elevatorLift.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
