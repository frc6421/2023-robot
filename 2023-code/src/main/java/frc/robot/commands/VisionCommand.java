// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;

  private String allianceColor;

  // private int tagID;

  // private double currentXPose;
  // private double currentYPose;
  // private double currentYawAngle;

  private Pose2d currentPose;

  private double xOffset;
  private double yOffset;

  private double tagXPose;
  private double tagYPose;

  private double targetXPose;
  private double targetYPose;
  private double targetYawAngle;

  private Pose2d targetPose;

  private Trajectory visionTrajectory;

  /** Creates a new VisionCommand. */
  public VisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    addRequirements(driveSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        VisionConstants.VISION_MAX_VELOCITY_METERS_PER_SECOND,
        VisionConstants.VISION_MAX_ACCEL_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics);

    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        VisionConstants.VISION_MAX_VELOCITY_METERS_PER_SECOND,
        VisionConstants.VISION_MAX_ACCEL_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics)
        .setReversed(true);

    allianceColor = DriverStation.getAlliance().name();

    // if (RobotContainer.robotState.equals(RobotStates.DRIVE) || RobotContainer.robotState.equals(RobotStates.INTAKE)) {
    //   tagID = 0;
    // }

    if (allianceColor.equals("Red")) {
      if (!RobotContainer.robotState.equals(RobotStates.LEFT_SUBSTATION)
          && !RobotContainer.robotState.equals(RobotStates.RIGHT_SUBSTATION)
          && !RobotContainer.robotState.equals(RobotStates.SINGLE_SUBSTATION)) {
        switch (RobotContainer.gridState) {
          // Red alliance left grid (driver perspective)
          case ONE:

            tagXPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_X + 0.3;
            tagYPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y;

            targetYawAngle = 0;

            break;
          // Red alliance center grid
          case TWO:

            tagXPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_X;
            tagYPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_Y;

            targetYawAngle = 0;

            break;
          // Red alliance right grid
          case THREE:

            tagXPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_X;
            tagYPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y;

            targetYawAngle = 0;

            break;
          // When no target is detected (no grid state selected)
          default:
            targetXPose = driveSubsystem.getPose2d().getX();
            targetYPose = driveSubsystem.getPose2d().getY();
            targetYawAngle = driveSubsystem.getPose2d().getRotation().getRadians();
            break;
        }

      } else if (RobotContainer.robotState.equals(RobotStates.LEFT_SUBSTATION) ||
          RobotContainer.robotState.equals(RobotStates.RIGHT_SUBSTATION)) {

        tagXPose = VisionConstants.RED_SUBSTATION_POSE_X;
        tagYPose = VisionConstants.RED_SUBSTATION_POSE_Y;

        targetYawAngle = Units.degreesToRadians(180);

      } else if (RobotContainer.robotState.equals(RobotStates.SINGLE_SUBSTATION)) {

        tagXPose = VisionConstants.RED_SUBSTATION_POSE_X;
        tagYPose = VisionConstants.RED_SUBSTATION_POSE_Y;

        targetYawAngle = Units.degreesToRadians(90);

        yOffset = VisionConstants.SINGLE_SUBSTATION_Y_OFFSET;

      }
    } else if (allianceColor.equals("Blue")) {
      if (!RobotContainer.robotState.equals(RobotStates.LEFT_SUBSTATION)
          && !RobotContainer.robotState.equals(RobotStates.RIGHT_SUBSTATION)
          && !RobotContainer.robotState.equals(RobotStates.SINGLE_SUBSTATION)) {
        switch (RobotContainer.gridState) {
          // Blue alliance left grid
          case ONE:

            tagXPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_X;
            tagYPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_Y;

            targetYawAngle = 0;

            break;
          // Blue alliance center grid
          case TWO:

            tagXPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_X;
            tagYPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_Y;

            targetYawAngle = 0;

            break;
          // Blue alliance right grid
          case THREE:

            tagXPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_X;
            tagYPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_Y;

            targetYawAngle = 0;

            break;
          // When no target is detected (no grid state selected)
          default:
            targetXPose = driveSubsystem.getPose2d().getX();
            targetYPose = driveSubsystem.getPose2d().getY();
            targetYawAngle = driveSubsystem.getPose2d().getRotation().getRadians();
            break;
        }
      } else if (RobotContainer.robotState.equals(RobotStates.LEFT_SUBSTATION) ||
          RobotContainer.robotState.equals(RobotStates.RIGHT_SUBSTATION)) {

        tagXPose = VisionConstants.BLUE_SUBSTATION_POSE_X;
        tagYPose = VisionConstants.BLUE_SUBSTATION_POSE_Y;

        targetYawAngle = Units.degreesToRadians(180);

      } else if (RobotContainer.robotState.equals(RobotStates.SINGLE_SUBSTATION)) {

        tagXPose = VisionConstants.BLUE_SUBSTATION_POSE_X;
        tagYPose = VisionConstants.BLUE_SUBSTATION_POSE_Y;

        targetYawAngle = Units.degreesToRadians(90);

        yOffset = -VisionConstants.SINGLE_SUBSTATION_Y_OFFSET;

      }

    }

    if (RobotContainer.robotState.equals(RobotStates.HIGH_LEFT)
        || RobotContainer.robotState.equals(RobotStates.MID_LEFT)
        || RobotContainer.robotState.equals(RobotStates.HYBRID_LEFT)) {

      xOffset = TrajectoryConstants.CENTER_OF_ROBOT_LENGTH + VisionConstants.GRID_OFFSET;
      yOffset = VisionConstants.CONE_OFFSET;

    } else if (RobotContainer.robotState.equals(RobotStates.HIGH_RIGHT)
        || RobotContainer.robotState.equals(RobotStates.MID_RIGHT)
        || RobotContainer.robotState.equals(RobotStates.HYBRID_RIGHT)) {

      xOffset = TrajectoryConstants.CENTER_OF_ROBOT_LENGTH + VisionConstants.GRID_OFFSET;
      yOffset = -VisionConstants.CONE_OFFSET;

    } else if (RobotContainer.robotState.equals(RobotStates.HIGH_CENTER)
        || RobotContainer.robotState.equals(RobotStates.MID_CENTER)
        || RobotContainer.robotState.equals(RobotStates.HYBRID_CENTER)) {

      xOffset = TrajectoryConstants.CENTER_OF_ROBOT_LENGTH + VisionConstants.GRID_OFFSET;
      yOffset = 0;

    } else if (RobotContainer.robotState.equals(RobotStates.LEFT_SUBSTATION)) {

      xOffset = -VisionConstants.SUBSTATION_X_OFFSET;
      yOffset = VisionConstants.SUBSTATION_Y_OFFSET;

    } else if (RobotContainer.robotState.equals(RobotStates.RIGHT_SUBSTATION)) {

      xOffset = -VisionConstants.SUBSTATION_X_OFFSET;
      yOffset = -VisionConstants.SUBSTATION_Y_OFFSET;

    } else if (RobotContainer.robotState.equals(RobotStates.SINGLE_SUBSTATION)) {

      xOffset = -VisionConstants.SINGLE_SUBSTATION_X_OFFSET;

    }

    targetXPose = tagXPose + xOffset;
    targetYPose = tagYPose + yOffset;

    SmartDashboard.putNumber("Target X", targetXPose);
    SmartDashboard.putNumber("Target Y", targetYPose);

    currentPose = driveSubsystem.getPose2d();
    targetPose = new Pose2d(new Translation2d(targetXPose, targetYPose), new Rotation2d(targetYawAngle));

    if (!RobotContainer.robotState.equals(RobotStates.DRIVE) ||
        !RobotContainer.robotState.equals(RobotStates.INTAKE)) {
      visionTrajectory = TrajectoryGenerator.generateTrajectory(List.of(currentPose,
          targetPose), reverseConfig);
    } else {
      visionTrajectory = null;
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

    SwerveControllerCommand visionDriveCommand = new SwerveControllerCommand(
        visionTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.swerveKinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    System.out.println(visionTrajectory.getStates());
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        visionDriveCommand,
        new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)));
  }
}
