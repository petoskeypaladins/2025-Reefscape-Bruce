// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LineUpRightCommand extends Command {

    double startEncoderCount;
   // double encoderDistance = 0.27911;
   double encoderDistance;
  /** Creates a new AlgaePivot. */
  public LineUpRightCommand(double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);

    encoderDistance = target;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    startEncoderCount = RobotContainer.m_robotDrive.m_frontLeft.getPosition().distanceMeters;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(RobotContainer.m_robotDrive.m_frontLeft.getPosition().distanceMeters - startEncoderCount) < encoderDistance) {
        RobotContainer.m_robotDrive.drive(0,-0.1, 0,false);
    }
    else {
        RobotContainer.m_robotDrive.drive(0,0, 0,false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(RobotContainer.m_robotDrive.m_frontLeft.getPosition().distanceMeters - startEncoderCount) >= encoderDistance) {
      return true;
    }
    else {
    return false;
    }
  }
}
