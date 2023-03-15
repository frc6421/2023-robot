// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceChargeCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ArmSubsystem armSubsystem;

  /**
   * Creates a new TwoPieceChargeCommand. Scores two pieces on the high row and
   * balances the charge station
   */
  public TwoPieceChargeCommand(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm) {
    driveSubsystem = drive;
    elevatorSubsystem = elevator;
    armSubsystem = arm;
    addRequirements(driveSubsystem, elevatorSubsystem, armSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics);

    TrajectoryConfig chargeConfig = new TrajectoryConfig(
        AutoConstants.AUTO_CHARGE_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_CHARGE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
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
        new Pose2d(TrajectoryConstants.FOURTH_GAME_PIECE, new Rotation2d(0))), forwardConfig);

    // Return to score cube next to cone
    Trajectory firstScoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FOURTH_GAME_PIECE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.CUBE_NODE, new Rotation2d(0))), reverseConfig);

    // Drives on charge station
    Trajectory chargeStationTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.CUBE_NODE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.COOPERTITION_CUBE_NODE.plus(new Translation2d(Units.inchesToMeters(36), 0)), new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.CENTER_OF_CHARGE_STATION, new Rotation2d(0))), chargeConfig);

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

    SwerveControllerCommand chargeStationCommand = new SwerveControllerCommand(
        chargeStationTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.swerveKinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> driveSubsystem.resetOdometry(firstPickUpTrajectory.getInitialPose())),
        // TODO determine which height we will score on in auto
        new ParallelCommandGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.HIGH),
            new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.HIGH)),
        // TODO turn on intake in deadline group
        new ParallelCommandGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE),
            new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE)),
        new WaitCommand(0.5),
        firstPickUpCommand,
        new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
        firstScoreCommand,
        new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
        new ParallelCommandGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.HIGH),
            new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.HIGH)),
        new ParallelCommandGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE),
            new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE)),
        new WaitCommand(0.5),
        chargeStationCommand,
        new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
        new BalanceCommand(driveSubsystem));
  }
}
