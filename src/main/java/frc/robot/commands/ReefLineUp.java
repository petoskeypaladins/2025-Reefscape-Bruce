// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefLineUp extends Command {
    // 7.76
    private double txGoal;
    private double tyGoal;
    private double taGoal;

  /** Creates a new CoralOuttakeCommand. */
  public ReefLineUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
    addRequirements(RobotContainer.m_LimelightSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
    if (RobotContainer.m_LimelightSubsystem.tv.getDouble(0) == 1) {
        if (RobotContainer.m_xboxController.getXButton()) {
            //Right D-pad pressed
            if (RobotContainer.m_xboxController.getPOV() == 90) {
               // txGoal = 
               // tyGoal =
            RobotContainer.m_LimelightSubsystem.RangeAim(txGoal, tyGoal, 0.5);
            }
            //Left D-pad pressed
            else if (RobotContainer.m_xboxController.getPOV() == 270) {
                txGoal = 6.75;
                tyGoal = -1.03;
            RobotContainer.m_LimelightSubsystem.RangeAim(txGoal, tyGoal, 0.5);
            }
            //Only X pressed
            else {
                txGoal = 0;
               // taGoal = 18.15;
               taGoal = 64;
            RobotContainer.m_LimelightSubsystem.RangeAimTa(txGoal, taGoal, 0.5);
            }
     }
    }
     */
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
