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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourPieceCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;

  private Field2d field;

  /**
   * Creates a new FourPieceCommand. Scores one full link on the high row and one
   * piece on the middle row. Right start.
   */
  public FourPieceCommand(DriveSubsystem drive) {
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

    // First action is scoring pre-loaded cone on high, second cone node

    // Pick up cube
    Trajectory pickUpTrajectoryOne = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.SECOND_CONE_NODE, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.FOURTH_GAME_PIECE, new Rotation2d(Units.degreesToRadians(0)))), forwardConfig);

    // Score cube (high)
    Trajectory scoreTrajectoryOne = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FOURTH_GAME_PIECE, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.FAR_EDGE_OF_COMMUNITY, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.CUBE_NODE, new Rotation2d(Units.degreesToRadians(0)))), reverseConfig);

    // Pick up second cone
    Trajectory pickUpTrajectoryTwo = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.CUBE_NODE, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.FAR_EDGE_OF_COMMUNITY, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.THIRD_GAME_PIECE, new Rotation2d(Units.degreesToRadians(0)))), forwardConfig);

    // Score second cone (high)
    Trajectory scoreTrajectoryTwo = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.THIRD_GAME_PIECE, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.FAR_EDGE_OF_COMMUNITY, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.FIRST_CONE_NODE, new Rotation2d(Units.degreesToRadians(0)))), reverseConfig);

    // Pick up third cone
    Trajectory pickUpTrajectoryThree = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FIRST_CONE_NODE, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.FAR_EDGE_OF_COMMUNITY, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.SECOND_GAME_PIECE, new Rotation2d(Units.degreesToRadians(0)))), forwardConfig);

    // Score third cone (mid)
    Trajectory scoreTrajectoryThree = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.SECOND_GAME_PIECE, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.FAR_EDGE_OF_COMMUNITY, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.SECOND_CONE_NODE, new Rotation2d(Units.degreesToRadians(0)))), reverseConfig);

    System.out.println(
        "Four Piece Auto Time: " + (pickUpTrajectoryOne.getTotalTimeSeconds() + scoreTrajectoryOne.getTotalTimeSeconds()
            + pickUpTrajectoryTwo.getTotalTimeSeconds() + scoreTrajectoryTwo.getTotalTimeSeconds()
            + pickUpTrajectoryThree.getTotalTimeSeconds() + scoreTrajectoryThree.getTotalTimeSeconds()));

    // Simulation
    field = new Field2d();

    if (RobotBase.isSimulation()) {
      SmartDashboard.putData(field);

    //   field.getObject("Pick Up Trajectory 1").setTrajectory(pickUpTrajectoryOne);
    //   field.getObject("Score Trajectory 1").setTrajectory(scoreTrajectoryOne);
    //   field.getObject("Pick Up Trajectory 2").setTrajectory(pickUpTrajectoryTwo);
    //   field.getObject("Score Trajectory 2").setTrajectory(scoreTrajectoryTwo);
    //   field.getObject("Pick Up Trajectory 3").setTrajectory(pickUpTrajectoryThree);
    //   field.getObject("Score Trajectory 3").setTrajectory(scoreTrajectoryThree);
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

      SwerveControllerCommand pickUpOneCommand = new SwerveControllerCommand(
          pickUpTrajectoryOne,
          driveSubsystem::getPose2d,
          driveSubsystem.swerveKinematics,
          holonomicDriveController,
          driveSubsystem::setModuleStates,
          driveSubsystem);

      SwerveControllerCommand scoreOneCommand = new SwerveControllerCommand(
          scoreTrajectoryOne,
          driveSubsystem::getPose2d,
          driveSubsystem.swerveKinematics,
          holonomicDriveController,
          driveSubsystem::setModuleStates,
          driveSubsystem);

      SwerveControllerCommand pickUpTwoCommand = new SwerveControllerCommand(
          pickUpTrajectoryTwo,
          driveSubsystem::getPose2d,
          driveSubsystem.swerveKinematics,
          holonomicDriveController,
          driveSubsystem::setModuleStates,
          driveSubsystem);

      SwerveControllerCommand scoreTwoCommand = new SwerveControllerCommand(
          scoreTrajectoryTwo,
          driveSubsystem::getPose2d,
          driveSubsystem.swerveKinematics,
          holonomicDriveController,
          driveSubsystem::setModuleStates,
          driveSubsystem);

      SwerveControllerCommand pickUpThreeCommand = new SwerveControllerCommand(
          pickUpTrajectoryThree,
          driveSubsystem::getPose2d,
          driveSubsystem.swerveKinematics,
          holonomicDriveController,
          driveSubsystem::setModuleStates,
          driveSubsystem);

      SwerveControllerCommand scoreThreeCommand = new SwerveControllerCommand(
          scoreTrajectoryThree,
          driveSubsystem::getPose2d,
          driveSubsystem.swerveKinematics,
          holonomicDriveController,
          driveSubsystem::setModuleStates,
          driveSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.resetOdometry(pickUpTrajectoryOne.getInitialPose())),
      pickUpOneCommand,
      new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, 0, 0)),
      scoreOneCommand,
      new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, 0, 0)),
      pickUpTwoCommand,
      new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, 0, 0)),
      scoreTwoCommand,
      new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, 0, 0)),
      pickUpThreeCommand,
      new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, 0, 0)),
      scoreThreeCommand,
      new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, 0, 0))
    );
  }
}
