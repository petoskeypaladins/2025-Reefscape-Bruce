// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlgaeArmOutAndSpin;
import frc.robot.commands.AlgaePivotDown;
import frc.robot.commands.AlgaePivotUp;
import frc.robot.commands.AlgaeReefRemove;
import frc.robot.commands.AlgaeReefRemoveReverse;
import frc.robot.commands.AlgaeReefWheels;
import frc.robot.commands.AlgaeSpinIn;
import frc.robot.commands.AlgaeSpinOut;
import frc.robot.commands.AutonCoralScoring;
import frc.robot.commands.AutonElevator;
import frc.robot.commands.AutonLineUp;
import frc.robot.commands.BackUpCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOuttake;
import frc.robot.commands.CoralOuttakeCommand;
import frc.robot.commands.CoralOuttakeReverseCommand;
import frc.robot.commands.Elevator;
import frc.robot.commands.LineUpForwardCommand;
import frc.robot.commands.LineUpRightCommand;
import frc.robot.commands.ReefLineUp;
import frc.robot.commands.ResetGyroCommand;
//import frc.robot.commands.resetGyro;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralScoringSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorAlgaeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.ElasticSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.util.List;

import javax.naming.PartialResultException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

// Limelight Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.LimelightHelpers;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  boolean L1Scheduler = false;
  boolean L2Scheduler = false;
  boolean L3Scheduler = false;
  boolean L4Scheduler = false;


 
  // The robot's subsystems:
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ElasticSubsystem m_ElasticSubsystem = new ElasticSubsystem();
  public final static ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  public final static FloorAlgaeSubsystem m_FloorAlgaeSubsystem = new FloorAlgaeSubsystem();
  public final static CoralScoringSubsystem m_CoralScoringSubsystem = new CoralScoringSubsystem();
  public final static Elastic m_Elastic = new Elastic();
  public final static LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
  public final static ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

     
    //The robot's commands:
    public final AlgaeSpinIn m_AlgaeSpinIn = new AlgaeSpinIn();
    public final AlgaeSpinOut m_AlgaeSpinOut = new AlgaeSpinOut();
    public final AlgaeSpinOut m_AlgaeSpinOut2 = new AlgaeSpinOut();
    public final AlgaeSpinOut m_AlgaeSpinOut3 = new AlgaeSpinOut();
    public final AlgaeSpinOut m_AutoAlgaeSpinOut = new AlgaeSpinOut();
    public final AlgaePivotUp m_AlgaePivotUp = new AlgaePivotUp();
    public final AlgaePivotDown m_AlgaePivotDown = new AlgaePivotDown();
    public final AlgaeReefWheels m_AlgaeReefWheels2 = new AlgaeReefWheels(); 
    public final CoralOuttakeCommand m_CoralScoring = new CoralOuttakeCommand();
    public final Elevator m_ElevatorCommand = new Elevator();
    private static final CoralOuttakeReverseCommand m_CoralScoringReverse = new CoralOuttakeReverseCommand();
    private final CoralIntakeCommand m_CoralIntake = new CoralIntakeCommand();
    private final CoralOuttake m_CoralOuttake = new CoralOuttake();
    private final AlgaeReefRemove m_AlgaeReefRemove = new AlgaeReefRemove();
    private final AlgaeReefRemove m_AlgaeReefRemove2 = new AlgaeReefRemove();
    private final AlgaeReefRemove m_AlgaeReefRemove3 = new AlgaeReefRemove(); 
    private final AlgaeReefWheels m_AlgaeReefWheels = new AlgaeReefWheels();
    private final AlgaeReefRemoveReverse m_AlgaeReefRemoveReverse = new AlgaeReefRemoveReverse();
    private final AlgaeArmOutAndSpin m_AlgaeArmOutAndSpin = new AlgaeArmOutAndSpin(); 
    private final ReefLineUp m_ReefLineUp = new ReefLineUp();
    private final AlgaeReefRemove m_AutonAlgaeReefRemove = new AlgaeReefRemove();
    private final AlgaeReefRemove m_ManualAlgaeReefRemove = new AlgaeReefRemove();
    //private final Command m_AutonResetGyro = new ResetGyroCommand(); 
    //line up for coral
    private final LineUpRightCommand m_LineUpRightCommand = new LineUpRightCommand(0.25);
    // line up for algae
    private final LineUpRightCommand m_LineUpAlgae = new LineUpRightCommand(0.125);
    private final LineUpRightCommand m_LineUpAlgae2 = new LineUpRightCommand(0.125);
    private final LineUpRightCommand m_AutonLineUpAlgae = new LineUpRightCommand(0.125);
    private final ClimbCommand m_ClimbCommand = new ClimbCommand();
    private final AutonLineUp m_AutonLineUp = new AutonLineUp();
    private final LineUpForwardCommand m_LineUpForwardCommand = new LineUpForwardCommand();
    private final BackUpCommand m_ReefBackUpCommand = new BackUpCommand();
    private final BackUpCommand m_AutonReefBackUpCommand = new BackUpCommand();

   // Elevator Heights
    private final AutonElevator m_L1AutonCommand = new AutonElevator(0);
    private final AutonElevator m_L2AutonCommand = new AutonElevator(10.275);
    private final AutonElevator m_L3AutonCommand = new AutonElevator(27.5);
    private final AutonElevator m_L3AutonCommand2 = new AutonElevator(27.5);

    private final AutonElevator m_L4AutonCommand = new AutonElevator(42); //  <-- Currently this is not the correct 
    //height for L4, but is instead being used to bump up the coral on L3. L4 was was 59.

    private final AutonElevator m_LowerAlgaeHeight = new AutonElevator(22.5);
    private final AutonElevator m_UpperAlgaeHeight = new AutonElevator(37.7);
    private final AutonElevator m_AutonUpperAlgaeHeight = new AutonElevator(37.7);
    private final AutonElevator m_AutonLowerAlgaeHeight = new AutonElevator(22.5);

    private final AutonCoralScoring m_AutonCoralScoring = new AutonCoralScoring();
    private final AutonCoralScoring m_AutonCoralScoring2 = new AutonCoralScoring();



    // The driver's controller
   public static XboxController m_xboxController = new XboxController(OIConstants.kDriverControllerPort);
   public static CommandXboxController m_commandXboxController = new CommandXboxController(OIConstants.kDriverControllerPort);
   public static Joystick m_Joystick = new Joystick(OIConstants.kOperatorPort);
   public static CommandJoystick m_CommandJoystick = new CommandJoystick(OIConstants.kOperatorPort);

  private final SendableChooser<Command> autoChooser;
  
   
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      // Configure the button bindings

      

      NamedCommands.registerCommand("Reset Gyro", new InstantCommand(
        () -> m_robotDrive.resetGyro()
      ));

      // Paths
      NamedCommands.registerCommand("Out of the Way", autonPath("Out of the Way"));
      NamedCommands.registerCommand("Reef to Barge", autonPath("Reef to Barge"));
      NamedCommands.registerCommand("Reef to Source", autonPath("Reef to Source"));
      NamedCommands.registerCommand("Source to Reef", autonPath("Source to Reef"));
      NamedCommands.registerCommand("Start Left to Reef Close", autonPath("Start Left to Reef Close"));
      NamedCommands.registerCommand("Start Mid to Reef Mid", autonPath("Start Mid to Reef Mid"));
      NamedCommands.registerCommand("Start Right Reef Close", autonPath("Start Right Reef Close"));
      NamedCommands.registerCommand("Limelight Forward Bottom", autonPath("Limelight Forward Bottom"));
      NamedCommands.registerCommand("Limelight Forward Middle", autonPath("Limelight Forward Middle"));
      NamedCommands.registerCommand("Limelight Forward Top", autonPath("Limelight Forward Top"));
      // NamedCommands.registerCommand("Start to Reef", autonPath("Start to Reef"));

      //Pathplanner Commands
      NamedCommands.registerCommand("Reef Line Up", m_AutonLineUp);
      NamedCommands.registerCommand("Move Right Algae", m_AutonLineUpAlgae);
      NamedCommands.registerCommand("Upper Algae", m_AutonUpperAlgaeHeight);
      NamedCommands.registerCommand("Lower Algae Height", m_AutonLowerAlgaeHeight); 
      NamedCommands.registerCommand("Algae Reef Remove", m_AutonAlgaeReefRemove);
      NamedCommands.registerCommand("Algae Spin Out", m_AutoAlgaeSpinOut);
      NamedCommands.registerCommand("Algae Reef Wheels", m_AlgaeReefWheels);
      NamedCommands.registerCommand("Reef Backup", m_AutonReefBackUpCommand);
      NamedCommands.registerCommand("Line Up Forward", m_LineUpForwardCommand);
      NamedCommands.registerCommand("L1", m_L1AutonCommand);
      NamedCommands.registerCommand("L2", m_L2AutonCommand);
      NamedCommands.registerCommand("L3", m_L3AutonCommand);
      NamedCommands.registerCommand("Score Coral", m_CoralScoring);
      //NamedCommands.registerCommand("reset gyro", m_AutonResetGyro);
      //NamedCommands.registerCommand("HeadingLeft", m_robotDrive.m_gyro.getAngle(Heading));
      // NamedCommands.registerCommand("Reset Gyro 180", Commands.runOnce(() -> m_robotDrive.resetGyro(180), m_robotDrive));

      configureButtonBindings();

     autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Select Auto", autoChooser);

      
  
      //triggers the command to intake algae from the floor
    //  XboxController.getRightTriggerAxis(m_AlgaeSpinIn);
      /*m_commandXboxController.axisGreaterThan(1, 0.5).whileTrue(m_AlgaeSpinIn);
      m_commandXboxController.leftTrigger(0.5).whileTrue(m_AlgaeSpinIn);
      m_commandXboxController.b().whileTrue(m_AlgaeSpinIn);*/
      
  }

  


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //feature from the template that makes the wheels into an "X" shape when you depress the right trigger
    /*new JoystickButton(m_xboxController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));"*/

      m_commandXboxController.rightBumper().whileTrue(m_AlgaeSpinIn);
      m_commandXboxController.leftBumper().whileTrue(m_AlgaeSpinOut); 
      m_commandXboxController.a().whileTrue(m_AlgaePivotUp);
      m_commandXboxController.b().whileTrue(m_AlgaePivotDown);
      //m_commandXboxController.x().whileTrue(m_ReefLineUp);
      m_commandXboxController.pov(90).onTrue(m_LineUpRightCommand);
      m_commandXboxController.pov(180).whileTrue(
        new RunCommand(
          () -> m_robotDrive.resetGyro(), m_robotDrive)
      );
      m_commandXboxController.x().onTrue(teleopLineUp());
      
     
     // 0.2794, 0.16764


      
      
  
      // m_CommandJoystick.button(1).whileTrue(m_CoralOuttakeCommand);
      m_CommandJoystick.button(1).whileTrue(m_CoralScoring);
      m_CommandJoystick.button(8).whileTrue(m_CoralScoringReverse);
      m_CommandJoystick.pov(180).whileTrue(m_CoralIntake);
      m_CommandJoystick.pov(0).whileTrue(m_CoralOuttake);
      // m_CommandJoystick.button(7).whileTrue(m_ClimbCommand);

      // Teleop Elevator

      // m_CommandJoystick.button(5).and(m_CommandJoystick.pov(90).whileFalse(m_L1AutonCommand));
      // m_CommandJoystick.button(3).and(m_CommandJoystick.pov(90).whileFalse(m_L2AutonCommand));
      // m_CommandJoystick.button(4).and(m_CommandJoystick.pov(90).whileFalse(m_L3AutonCommand));
      // m_CommandJoystick.button(6).and(m_CommandJoystick.pov(90).whileFalse(m_L4AutonCommand));
      // m_CommandJoystick.button(10).and(m_CommandJoystick.pov(90).whileFalse(m_AlgaeHeightAutonCommand));

      //m_CommandJoystick.button(5).onTrue(m_L1AutonCommand);

      // Auto-Scoring Teleop
      // m_CommandJoystick.pov(90)
      //   .and(m_CommandJoystick.button(3))
      //   .onTrue(teleopL2());
      m_CommandJoystick.pov(90).onTrue(teleopL2());
      m_CommandJoystick.pov(270).onTrue(teleopL3());


      m_CommandJoystick.button(11).whileTrue(m_AlgaeReefRemoveReverse);
      //m_CommandJoystick.button(7).whileTrue(m_ManualAlgaeReefRemove);
      m_CommandJoystick.button(7).whileTrue(m_AlgaeReefRemove);
      m_CommandJoystick.button(9).onTrue(teleopAlgae2Removal());
      m_CommandJoystick.button(10).whileTrue(m_AlgaeReefRemove);
      //m_CommandJoystick.button(12).whileTrue(m_AlgaeReefWheels);
      m_commandXboxController.leftTrigger(0.01).whileTrue(
        new RunCommand(
          () -> m_robotDrive.slowDrive(
              -MathUtil.applyDeadband(m_xboxController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_xboxController.getLeftX(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_xboxController.getRightX(), OIConstants.kDriveDeadband),
              m_robotDrive.isFieldOriented),
          m_robotDrive));

      

      // m_CommandJoystick.axisGreaterThan(1, 0.02).whileTrue(m_ElevatorCommand);
      // m_CommandJoystick.axisLessThan(1, -0.02).whileTrue(m_ElevatorCommand);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_xboxController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_xboxController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_xboxController.getRightX(), OIConstants.kDriveDeadband),
                m_robotDrive.isFieldOriented),
            m_robotDrive));

    m_ElevatorSubsystem.setDefaultCommand(m_ElevatorCommand);
            

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      // return Autos.exampleAuto(m_exampleSubsystem);
  
      return autoChooser.getSelected();
          
    }
  
  public Command teleopLineUp() {
    return new SequentialCommandGroup(
      m_AutonLineUp,
      new ParallelRaceGroup(
        m_LineUpForwardCommand,
        new WaitCommand(1)
      )
    );
  }

  public Command teleopL2() {
    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            new ParallelRaceGroup(
                m_L2AutonCommand,
                new WaitCommand(1.5)
        ),
          new SequentialCommandGroup(
              new WaitCommand(0.5),
              new ParallelRaceGroup(
                  new WaitCommand(0.8),
                  m_AutonCoralScoring
            )
          )
      ),
      new ParallelRaceGroup(
          m_L3AutonCommand,
          new WaitCommand(0.5)
      )
    );
  }

  
  public Command teleopL3() {
    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            new ParallelRaceGroup(
                m_L3AutonCommand2,
                new WaitCommand(2)
        ),
          new SequentialCommandGroup(
              new WaitCommand(0.8),
              new ParallelRaceGroup(
                  new WaitCommand(0.8),
                  m_AutonCoralScoring2
            )
          )
      ),
      
      new ParallelRaceGroup(
          m_L4AutonCommand,
              new WaitCommand(0.5)
      )

  
    );
  }
 

   /*Upper algae test code
  public Command teleopAlgae2Removal() {
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        m_LineUpAlgae,
        new WaitCommand(0.2)
      ),

      new SequentialCommandGroup(
        new ParallelRaceGroup(
        m_UpperAlgaeHeight,
         new WaitCommand(0.5)
        ),
        new ParallelRaceGroup(
        m_AlgaeReefRemove,
          new WaitCommand(0.75)
        ),
        new ParallelRaceGroup(
        m_AlgaeSpinOut,
          new WaitCommand(0.5)
        ),
        new ParallelRaceGroup(
        m_AlgaeSpinOut2,
        m_ReefBackUpCommand,
          new WaitCommand(1)
        )
        /*new ParallelRaceGroup(
        m_L1AutonCommand,
          new WaitCommand(1.5)
        ) 
      )
    );
  }*/

  //Button 9
  public Command teleopAlgae2Removal(){
    return new SequentialCommandGroup(
      m_LineUpAlgae,
      m_UpperAlgaeHeight,
      m_AlgaeReefRemove2
    );
  }

  //Button ??? 10???
  public Command teleopAlgae3Removal() {
    return new SequentialCommandGroup(
        m_LineUpAlgae2,
        m_LowerAlgaeHeight,
        m_AlgaeReefRemove3,
        m_AlgaeReefWheels2
    );
  }

  // public Command teleopAlgae1Removal(){
  //   return new SequentialCommandGroup(
  //     m_LineUpAlgae,
  //     m_LowerAlgaeHeight,
  //     m_AlgaeReefRemove2
  //   );
  // }



  public Command autonPath(String pathname) {
    PathPlannerPath path = null;
      try {
        path = PathPlannerPath.fromPathFile(pathname);
    }
     catch (IOException | ParseException e) {
      // Auto-generated catch block
      e.printStackTrace();
     }

     boolean isConfigured = AutoBuilder.isConfigured();
     if (isConfigured) 
        System.out.println(pathname + " is configured");
     else
        System.out.println(pathname + " is not configured");
  
    return AutoBuilder.followPath(path);
  }

  /* 
  public Command trajectory(double x, double y) {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0, 0), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(x,y, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    
  }
    */
}
