// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class VisionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private String allianceColor = DriverStation.getAlliance().name();
  private String limelightHostName = "limelight-two";

  private int tagID;

  private double currentXPose;
  private double currentYPose;
  private double currentYawAngle;

  private double targetXPose;
  private double targetYPose;
  private double targetYawAngle;

  private double xPValue;
  private double yPValue;
  private double yawPValue;

  private double allowableXError = 0.02;
  private double allowableYError = 0.02;
  private double allowableYawError = 1;

  private double xPercentAdjust;
  private double yPercentAdjust;
  private double yawPercentAdjust;

  private final TrapezoidProfile.Constraints translationConstraints;
  private final TrapezoidProfile.Constraints rotationConstraints;

  private final ProfiledPIDController xProfiledPIDController;
  private final ProfiledPIDController yProfiledPIDController;
  private final ProfiledPIDController yawProfiledPIDController;

  /** Creates a new VisionCommand. */
  public VisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    translationConstraints = new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    rotationConstraints = new TrapezoidProfile.Constraints(Units.radiansToDegrees(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC),
        Units.radiansToDegrees(AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));

    xProfiledPIDController = new ProfiledPIDController(xPValue, 0, 0, translationConstraints);
    yProfiledPIDController = new ProfiledPIDController(yPValue, 0, 0, translationConstraints);
    yawProfiledPIDController = new ProfiledPIDController(yawPValue, 0, 0, rotationConstraints);

    xProfiledPIDController.setTolerance(allowableXError);
    yProfiledPIDController.setTolerance(allowableYError);
    yawProfiledPIDController.setTolerance(allowableYawError);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tagID = (int) LimelightSubsystem.getAprilTagID(limelightHostName);

    switch (tagID) {
      // Red alliance left grid (driver perspective)
      case 1:
        if (RobotContainer.isLeftCone) {
          targetXPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isRightCone) {
          targetXPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isCube) {
          targetXPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }

        break;
      // Red alliance center grid
      case 2:
        if (RobotContainer.isLeftCone) {
          targetXPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isRightCone) {
          targetXPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isCube) {
          targetXPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }

        break;
      // Red alliance right grid
      case 3:
        if (RobotContainer.isLeftCone) {
          targetXPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isRightCone) {
          targetXPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isCube) {
          targetXPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }

        break;
      // Blue alliance substation
      // TODO test y offset
      // TODO test x offset with new intake
      case 4:
        if (RobotContainer.isLeftSubstation) {
          targetXPose = VisionConstants.BLUE_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.BLUE_SUBSTATION_POSE_Y + VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 0;

        } else if (!RobotContainer.isLeftSubstation) {
          targetXPose = VisionConstants.BLUE_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.BLUE_SUBSTATION_POSE_Y - VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 0;

        }

        break;
      // Red alliance substation
      case 5:
        if (RobotContainer.isLeftSubstation) {
          targetXPose = VisionConstants.RED_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.RED_SUBSTATION_POSE_Y + VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 0;

        } else if (!RobotContainer.isLeftSubstation) {
          targetXPose = VisionConstants.RED_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.RED_SUBSTATION_POSE_Y - VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 0;

        }

        break;
      // Blue alliance left grid
      case 6:
        if (RobotContainer.isLeftCone) {
          targetXPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isRightCone) {
          targetXPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isCube) {
          targetXPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }
        break;
      // Blue alliance center grid
      case 7:
        if (RobotContainer.isLeftCone) {
          targetXPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isRightCone) {
          targetXPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isCube) {
          targetXPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }
        break;
      // Blue alliance right grid
      case 8:
        if (RobotContainer.isLeftCone) {
          targetXPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isRightCone) {
          targetXPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.isCube) {
          targetXPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }
        break;
      default:
        end(true);
        break;
    }

    xProfiledPIDController.setGoal(targetXPose);
    yProfiledPIDController.setGoal(targetYPose);
    yawProfiledPIDController.setGoal(targetYawAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (allianceColor == "Red") {
      currentXPose = LimelightSubsystem.getRedBotPoseX(limelightHostName);
      currentYPose = LimelightSubsystem.getRedBotPoseY(limelightHostName);
      currentYawAngle = LimelightSubsystem.getRedBotPoseYaw(limelightHostName);
    } else if (allianceColor == "Blue") {
      currentXPose = LimelightSubsystem.getBlueBotPoseX(limelightHostName);
      currentYPose = LimelightSubsystem.getBlueBotPoseY(limelightHostName);
      currentYawAngle = LimelightSubsystem.getBlueBotPoseYaw(limelightHostName);
    }

    xPercentAdjust = MathUtil.clamp(xProfiledPIDController.calculate(currentXPose), -1, 1);
    yPercentAdjust = MathUtil.clamp(yProfiledPIDController.calculate(currentYPose), -1, 1);
    yawPercentAdjust = MathUtil.clamp(yawProfiledPIDController.calculate(currentYawAngle), -1, 1);

    //driveSubsystem.autoDrive(xPercentAdjust, yPercentAdjust, yawPercentAdjust);
    driveSubsystem.autoDrive(xPercentAdjust, 0, 0);
    //driveSubsystem.autoDrive(0, yPercentAdjust, 0);
    //driveSubsystem.autoDrive(0, 0, yawPercentAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.autoDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return currentXPose <= allowableXError && currentYPose <= allowableYError && currentYawAngle <= allowableYawError;
    return currentXPose <= allowableXError;
    //return currentYPose <= allowableYError;
    //return currentYawAngle <= allowableYawError;
  }
}
