// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonLineUp extends Command {
    // 7.76
    private double txGoal = 0;
    private double tyGoal;
    private double taGoal = 26;

    double initialLimelight;

  /** Creates a new CoralOuttakeCommand. */
  public AutonLineUp() {
    // Use addRequirements() here to declarep- subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
    addRequirements(RobotContainer.m_LimelightSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialLimelight = RobotContainer.m_LimelightSubsystem.tv.getDouble(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_LimelightSubsystem.tv.getDouble(0) == 1)
   RobotContainer.m_LimelightSubsystem.RangeAimTaAuton(txGoal, taGoal, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(taGoal - RobotContainer.m_LimelightSubsystem.area) < 0.5 || initialLimelight == 0 ) {
        return true;
    }
    else {
        return false;
    }
  }
}
