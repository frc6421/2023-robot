// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
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

  private final double xPValue = 1.2;
  private final double yPValue = 1.2;
  private final double yawPValue = 0.015;

  private final double allowableXError = 0.01;
  private final double allowableYError = 0.01;
  private final double allowableYawError = 0.5;

  private double xPercentAdjust;
  private double yPercentAdjust;
  private double yawPercentAdjust;

  private final PIDController xPIDController;
  private final PIDController yPIDController;
  private final PIDController yawPIDController;

  /** Creates a new VisionCommand. */
  public VisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    xPIDController = new PIDController(xPValue, 0, 0);
    yPIDController = new PIDController(yPValue, 0, 0);
    yawPIDController = new PIDController(yawPValue, 0, 0);

    xPIDController.setTolerance(allowableXError);
    yPIDController.setTolerance(allowableYError);
    yawPIDController.setTolerance(allowableYawError);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(LimelightSubsystem.isTargetDetected(limelightHostName)) {
      tagID = (int)LimelightSubsystem.getAprilTagID(limelightHostName);
    } else {
      end(true);
    }

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
          targetYawAngle = 180;

        } else if (!RobotContainer.isLeftSubstation) {
          targetXPose = VisionConstants.BLUE_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.BLUE_SUBSTATION_POSE_Y - VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 180;

        }

        break;
      // Red alliance substation
      case 5:
        if (RobotContainer.isLeftSubstation) {
          targetXPose = VisionConstants.RED_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.RED_SUBSTATION_POSE_Y + VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 180;

        } else if (!RobotContainer.isLeftSubstation) {
          targetXPose = VisionConstants.RED_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.RED_SUBSTATION_POSE_Y - VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 180;

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

    xPIDController.setSetpoint(targetXPose);
    yPIDController.setSetpoint(targetYPose);
    yawPIDController.setSetpoint(targetYawAngle);

    // System.out.println("X Target: " + targetXPose);
    // System.out.println("Y Target: " + targetYPose);
    // System.out.println("Yaw Target: " + targetYawAngle);

    if (allianceColor == "Red") {
      driveSubsystem.resetOdometry(new Pose2d(LimelightSubsystem.getRedBotPoseX(limelightHostName),
          LimelightSubsystem.getRedBotPoseY(limelightHostName), GyroSubsystem.getYawAngle()));

    } else if (allianceColor == "Blue") {
      driveSubsystem.resetOdometry(new Pose2d(LimelightSubsystem.getBlueBotPoseX(limelightHostName),
          LimelightSubsystem.getBlueBotPoseY(limelightHostName), GyroSubsystem.getYawAngle()));

    }

    // System.out.println("Total robot pose: " +
    // driveSubsystem.getPose2d().toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      currentXPose = driveSubsystem.getPose2d().getX();
      currentYPose = driveSubsystem.getPose2d().getY();
      currentYawAngle = driveSubsystem.getPose2d().getRotation().getDegrees();

      // System.out.println("Current X: " + currentXPose);
      // System.out.println("Current X: " + currentXPose);
      // System.out.println("X Target: " + targetXPose);

      xPercentAdjust = MathUtil.clamp(xPIDController.calculate(currentXPose), -1, 1);
      yPercentAdjust = MathUtil.clamp(yPIDController.calculate(currentYPose), -1, 1);
      yawPercentAdjust = MathUtil.clamp(yawPIDController.calculate(currentYawAngle), -1, 1);

      SmartDashboard.putNumber("xPercentAdjust", xPercentAdjust);
      SmartDashboard.putNumber("yPercentAdjust", yPercentAdjust);
      SmartDashboard.putNumber("yawPercentAdjust", yawPercentAdjust);

      SmartDashboard.putNumber("Current X", currentXPose);
      SmartDashboard.putNumber("Current Y", currentYPose);
      SmartDashboard.putNumber("Current Yaw", currentYawAngle);

      SmartDashboard.putNumber("Target X", targetXPose);
      SmartDashboard.putNumber("Target Y", targetYPose);
      SmartDashboard.putNumber("Target Yaw", targetYawAngle);

      driveSubsystem.autoDrive(xPercentAdjust, yPercentAdjust, yawPercentAdjust);
      // driveSubsystem.autoDrive(xPercentAdjust, yPercentAdjust, 0);
      // driveSubsystem.autoDrive(0, yPercentAdjust, 0);
      // driveSubsystem.autoDrive(0, 0, yawPercentAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      System.out.println("interrupted");
    }

    driveSubsystem.autoDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPIDController.atSetpoint() && yPIDController.atSetpoint() && yawPIDController.atSetpoint();
    // return xPIDController.atSetpoint();
    // return yPIDController.atSetpoint();
    // return yawPIDController.atSetpoint();
  }
}
