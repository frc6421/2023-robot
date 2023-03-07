// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmElevatorCommand;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.ArmElevatorCommand.PlaceStates;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeArmCommand.IntakePlaceStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ArmSubsystem armSubsystem;
  private GrabberSubsystem grabberSubsystem;
  private IntakeSubsystem intakeSubsystem;

  private Field2d field;

  /** Creates a new TwoPieceCommand. */
  public TwoPieceCommand(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm, GrabberSubsystem grabber, IntakeSubsystem intake) {
    driveSubsystem = drive;
    elevatorSubsystem = elevator;
    armSubsystem = arm;
    grabberSubsystem = grabber;
    intakeSubsystem = intake;
    addRequirements(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem, intakeSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics);

    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics)
        .setReversed(true);

    // Starts at far right cone node
    // Stage cube in far right game piece
    Trajectory firstPickUpTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(TrajectoryConstants.SECOND_CONE_NODE, new Rotation2d(0)),
      new Pose2d(TrajectoryConstants.FOURTH_GAME_PIECE, new Rotation2d(0))
    ), forwardConfig);

    // Return to score cube next to cone
    Trajectory firstScoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(TrajectoryConstants.FOURTH_GAME_PIECE, new Rotation2d(0)),
      new Pose2d(TrajectoryConstants.AROUND_CHARGE_STATION, new Rotation2d(0)),
      new Pose2d(TrajectoryConstants.CUBE_NODE, new Rotation2d(0))
    ), reverseConfig);

    // Gets the robot in a good position to start tele-op
    Trajectory edgeOfCommunityTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(TrajectoryConstants.CUBE_NODE, new Rotation2d(0)),
      new Pose2d(TrajectoryConstants.AROUND_CHARGE_STATION, new Rotation2d(0)),
      new Pose2d(TrajectoryConstants.FAR_EDGE_OF_COMMUNITY, new Rotation2d(0))
    ), forwardConfig);

    // Simulation
    field = new Field2d();

    if (RobotBase.isSimulation()) {
      SmartDashboard.putData(field);

      field.setRobotPose(firstPickUpTrajectory.getInitialPose());
      
      field.getObject("Pick Up Trajectory 1").setTrajectory(firstPickUpTrajectory);
      field.getObject("Score Trajectory 1").setTrajectory(firstScoreTrajectory);
      field.getObject("Out of Community Trajectory").setTrajectory(edgeOfCommunityTrajectory);
    }


    var thetaController = new ProfiledPIDController(
                AutoConstants.THETA_P, AutoConstants.THETA_I, AutoConstants.THETA_D,
                new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
                        AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
                // Position controllers
                new PIDController(AutoConstants.X_DRIVE_P, AutoConstants.X_DRIVE_I, AutoConstants.X_DRIVE_D),
                new PIDController(AutoConstants.Y_DRIVE_P, AutoConstants.Y_DRIVE_I, AutoConstants.Y_DRIVE_D),
                thetaController);

        SwerveControllerCommand firstPickUpCommand = new SwerveControllerCommand(
                firstPickUpTrajectory,
                driveSubsystem::getPose2d,
                driveSubsystem.swerveKinematics,
                holonomicDriveController,
                driveSubsystem::autoSetModuleStates,
                driveSubsystem);

        SwerveControllerCommand firstScoreCommand = new SwerveControllerCommand(
                firstScoreTrajectory,
                driveSubsystem::getPose2d,
                driveSubsystem.swerveKinematics,
                holonomicDriveController,
                driveSubsystem::autoSetModuleStates,
                driveSubsystem);

        SwerveControllerCommand edgeOfCommunityCommand = new SwerveControllerCommand(
                edgeOfCommunityTrajectory,
                driveSubsystem::getPose2d,
                driveSubsystem.swerveKinematics,
                holonomicDriveController,
                driveSubsystem::autoSetModuleStates,
                driveSubsystem);
                
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.resetOdometry(firstPickUpTrajectory.getInitialPose())),
      new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.HIGH),
      new ParallelDeadlineGroup(new WaitCommand(0.7), new InstantCommand(() -> grabberSubsystem.release())),
      new ParallelDeadlineGroup(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.FLOOR), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(1))),
      new ParallelCommandGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE), new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE)),
      firstPickUpCommand,
      new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
      new ParallelDeadlineGroup(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.DRIVE), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0.2))),
      firstScoreCommand,
      new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
      // Start of transfer sequence
      new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.TRANSFER),
      new ArmCommand(armSubsystem, ArmCommand.PlaceStates.TRANSFER),
      new ParallelDeadlineGroup(new WaitCommand(0.6), new InstantCommand(() -> grabberSubsystem.grab())),
      // End of transfer sequence
      new ParallelCommandGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.HIGH), new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.HIGH)),
      new ParallelDeadlineGroup(new WaitCommand(0.7), new InstantCommand(() -> grabberSubsystem.release())),
      edgeOfCommunityCommand,
      new ParallelCommandGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE), new ElevatorCommand(elevator, ElevatorCommand.PlaceStates.DRIVE)),
      new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0))
    );
  }
}
