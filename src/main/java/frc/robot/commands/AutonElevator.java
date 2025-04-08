// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonElevator extends Command {

  double elevatorTarget;
  /** Creates a new CoralOuttakeCommand. */
  public AutonElevator(double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ElevatorSubsystem);

    elevatorTarget = -target;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    RobotContainer.m_ElevatorSubsystem.elevatorLift.setControl(m_request.withPosition(elevatorTarget));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
