package frc.robot.commands;

import frc.robot.RobotContainer;
import java.lang.annotation.Target;
import edu.wpi.first.wpilibj2.command.Command;

public class BackUpCommand extends Command {
   
  public BackUpCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        RobotContainer.m_robotDrive.drive(-0.07,0, 0,false);
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_robotDrive.drive(0,0, 0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    }
}


