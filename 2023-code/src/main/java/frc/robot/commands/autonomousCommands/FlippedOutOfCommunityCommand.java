// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.commands.ArmElevatorCommand;
import frc.robot.commands.ArmElevatorCommand.PlaceStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlippedOutOfCommunityCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ArmSubsystem armSubsystem;
  private GrabberSubsystem grabberSubsystem;
  /** Creates a new FlippedOutOfCommunityCommand. */
  public FlippedOutOfCommunityCommand(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm, GrabberSubsystem grabber) {
    driveSubsystem = drive;
    elevatorSubsystem = elevator;
    armSubsystem = arm;
    grabberSubsystem = grabber;
    addRequirements(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics);

    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics)
        .setReversed(true);

        Trajectory outOfCommunityTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FLIPPED_SECOND_CONE_NODE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FLIPPED_FOURTH_GAME_PIECE, new Rotation2d(0))), forwardConfig);
    
    var xPIDController = new PIDController(AutoConstants.X_DRIVE_P, AutoConstants.X_DRIVE_I, AutoConstants.X_DRIVE_D);
    var yPIDController = new PIDController(AutoConstants.Y_DRIVE_P, AutoConstants.Y_DRIVE_I, AutoConstants.Y_DRIVE_D);

    var thetaController = new ProfiledPIDController(
        AutoConstants.THETA_P, AutoConstants.THETA_I, AutoConstants.THETA_D,
        new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
            AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand outOfCommunityCommand = new SwerveControllerCommand(
        outOfCommunityTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.swerveKinematics,
        xPIDController,
        yPIDController,
        thetaController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.HIGH),
        new ParallelDeadlineGroup(new WaitCommand(0.7), new InstantCommand(() -> grabberSubsystem.toggleGrabber())),
        new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.UP),
        new InstantCommand(() -> driveSubsystem.resetOdometry(outOfCommunityTrajectory.getInitialPose())),
        outOfCommunityCommand,
        new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, 0, 0)));
  }
}
