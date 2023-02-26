// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ScoringVisionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private String limelightHostName = "limelight-two";

  private int targetTagID;

  private Pose2d currentRobotPose;
  private Pose2d targetRobotPose;

  private Timer timer;

  private TrajectoryConfig config;
  private Trajectory scoreTrajectory;
  private ProfiledPIDController thetaController;
  private HolonomicDriveController holonomicDriveController;
  private SwerveControllerCommand scoreCommand;
  private SequentialCommandGroup scoreCommandGroup;

  private double xReflectAngle;
  private double targetReflectAngle = 0;
  private double reflectAngleError;
  private double allowableReflectAngleError = 0.5;
  private double reflectPercentAdjust;
  private double reflectP = 0.01; // TODO update with constant

  /** Creates a new ScoringVisionCommand. */
  public ScoringVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();

    if (DriverStation.getAlliance() == Alliance.Red) {
      currentRobotPose = LimelightHelpers.getBotPose2d_wpiRed(limelightHostName);
    } else if (DriverStation.getAlliance() == Alliance.Blue) {
      currentRobotPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightHostName);
    }

    config = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics);

    driveSubsystem.resetOdometry(currentRobotPose);
    targetTagID = (int) LimelightSubsystem.getAprilTagID(limelightHostName);

    if (DriverStation.getAlliance() == Alliance.Red) {
      switch (targetTagID) {
        // Left grid (from driver perspective) on red alliance
        case 1:
          if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_YELLOW) {
            if (RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.RED_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            } else if (!RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.RED_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            }
          } else if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_VIOLET) {
            targetRobotPose = new Pose2d(
                (VisionConstants.RED_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                    + VisionConstants.GRID_OFFSET),
                (VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y),
                new Rotation2d(0));

            scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                currentRobotPose,
                targetRobotPose), config);
          }
          break;
        // Middle grid (from driver perspective) on red alliance
        case 2:
          if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_YELLOW) {
            if (RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.RED_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.RED_CENTER_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            } else if (!RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.RED_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.RED_CENTER_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            }
          } else if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_VIOLET) {
            targetRobotPose = new Pose2d(
                (VisionConstants.RED_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                    + VisionConstants.GRID_OFFSET),
                (VisionConstants.RED_CENTER_GRID_CUBE_POSE_Y),
                new Rotation2d(0));

            scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                currentRobotPose,
                targetRobotPose), config);
          }
          break;
        // Right grid (from driver perspective) on red alliance
        case 3:
          if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_YELLOW) {
            if (RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.RED_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.RED_RIGHT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            } else if (!RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.RED_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.RED_RIGHT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            }
          } else if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_VIOLET) {
            targetRobotPose = new Pose2d(
                (VisionConstants.RED_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                    + VisionConstants.GRID_OFFSET),
                (VisionConstants.RED_RIGHT_GRID_CUBE_POSE_Y),
                new Rotation2d(0));

            scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                currentRobotPose,
                targetRobotPose), config);
          }
          break;
      }

    } else if (DriverStation.getAlliance() == Alliance.Blue) {
      switch (targetTagID) {
        // Left grid (from driver perspective) on blue alliance
        case 6:
          if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_YELLOW) {
            if (RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            } else if (!RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            }
          } else if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_VIOLET) {
            targetRobotPose = new Pose2d(
                (VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                    + VisionConstants.GRID_OFFSET),
                (VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_Y),
                new Rotation2d(0));

            scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                currentRobotPose,
                targetRobotPose), config);
          }
          break;
        // Middle grid (from driver perspective) on blue alliance
        case 7:
          if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_YELLOW) {
            if (RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            } else if (!RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            }
          } else if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_VIOLET) {
            targetRobotPose = new Pose2d(
                (VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                    + VisionConstants.GRID_OFFSET),
                (VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_Y),
                new Rotation2d(0));

            scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                currentRobotPose,
                targetRobotPose), config);
          }
          break;
        // Right grid (from driver perspective) on blue alliance
        case 8:
          if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_YELLOW) {
            if (RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            } else if (!RobotContainer.isLeftCone) {
              targetRobotPose = new Pose2d(
                  (VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                      + VisionConstants.GRID_OFFSET),
                  (VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET),
                  new Rotation2d(0));

              scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                  currentRobotPose,
                  targetRobotPose), config);
            }
          } else if (BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_VIOLET) {
            targetRobotPose = new Pose2d(
                (VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
                    + VisionConstants.GRID_OFFSET),
                (VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_Y),
                new Rotation2d(0));

            scoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                currentRobotPose,
                targetRobotPose), config);
          }
          break;
      }
    }

    thetaController = new ProfiledPIDController(
        AutoConstants.THETA_P, AutoConstants.THETA_I, AutoConstants.THETA_D,
        new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
            AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    holonomicDriveController = new HolonomicDriveController(
        // Position controllers
        new PIDController(AutoConstants.X_DRIVE_P, AutoConstants.X_DRIVE_I, AutoConstants.X_DRIVE_D),
        new PIDController(AutoConstants.Y_DRIVE_P, AutoConstants.Y_DRIVE_I, AutoConstants.Y_DRIVE_D),
        thetaController);

    scoreCommand = new SwerveControllerCommand(
        scoreTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.swerveKinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    scoreCommandGroup.addCommands(scoreCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.autoDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > scoreTrajectory.getTotalTimeSeconds();
  }
}
