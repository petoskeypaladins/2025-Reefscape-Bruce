// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeReefRemove extends Command {
  /** Creates a new AlgaeReefRemove. */
  public AlgaeReefRemove() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((RobotContainer.m_FloorAlgaeSubsystem.algaeOutLimitSwitch.get() == true && RobotContainer.m_ElevatorSubsystem.elevatorLift.getPosition().getValueAsDouble() < -5))
  
    //Previously -0.8
    RobotContainer.m_FloorAlgaeSubsystem.algaeRemoval.set(-1);

   else {
  RobotContainer.m_FloorAlgaeSubsystem.algaeRemoval.set(0);
}

//I set the speed of the algae removal wheels here outside of if statement
RobotContainer.m_FloorAlgaeSubsystem.algaeRemovalSpinner.set(0.5); 

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_FloorAlgaeSubsystem.algaeRemoval.set(0);
    //RobotContainer.m_FloorAlgaeSubsystem.algaeRemovalSpinner.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.m_FloorAlgaeSubsystem.algaeOutLimitSwitch.get() == false) 
      return true;
    else {
      return false;
    }
  }
}
