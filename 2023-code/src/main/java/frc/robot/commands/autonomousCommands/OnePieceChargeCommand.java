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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceChargeCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;

  /** Creates a new OnePieceChargeCommand. */
  public OnePieceChargeCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    addRequirements(driveSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics);

    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics)
        .setReversed(true);

    // Starts in line with the right cone node in co-op grid
    Trajectory chargeStationTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.SECOND_COOPERTITION_CONE_NODE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FAR_EDGE_OF_COMMUNITY, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.MID_POINT_OF_PIECES_AND_CHARGE_STATION, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.CENTER_OF_CHARGE_STATION, new Rotation2d(0))), forwardConfig);

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

    SwerveControllerCommand chargeStationCommand = new SwerveControllerCommand(
        chargeStationTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.swerveKinematics,
        holonomicDriveController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.resetOdometry(chargeStationTrajectory.getInitialPose())),
      chargeStationCommand,
      new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, 0, 0)));
  }
}
