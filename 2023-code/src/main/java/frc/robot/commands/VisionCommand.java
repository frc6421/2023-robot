// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class VisionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private String allianceColor;

  private int tagID;

  private double currentXPose;
  private double currentYPose;
  private double currentYawAngle;

  private double targetXPose;
  private double targetYPose;
  private double targetYawAngle;

  private final double xPValue = 4.9;
  private final double yPValue = 5;
  private final double yawPValue = 0.04;

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
    allianceColor = DriverStation.getAlliance().name();

    if(LimelightSubsystem.isTargetDetected("limelight-two")) {
      tagID = (int)LimelightSubsystem.getAprilTagID("limelight-two");
      System.out.println(tagID);
    } else {
      CommandScheduler.getInstance().cancel(this);
    }


    switch (tagID) {
      // Red alliance left grid (driver perspective)
      case 1:
        if (RobotContainer.robotState == RobotStates.HIGH_LEFT || RobotContainer.robotState == RobotStates.MID_LEFT) {
          targetXPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_RIGHT || RobotContainer.robotState == RobotStates.MID_RIGHT) {
          targetXPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_CENTER || RobotContainer.robotState == RobotStates.MID_CENTER) {
          targetXPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_LEFT_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }

        break;
      // Red alliance center grid
      case 2:
        if (RobotContainer.robotState == RobotStates.HIGH_LEFT || RobotContainer.robotState == RobotStates.MID_LEFT) {
          targetXPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_RIGHT || RobotContainer.robotState == RobotStates.MID_RIGHT) {
          targetXPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_CENTER || RobotContainer.robotState == RobotStates.MID_CENTER) {
          targetXPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_CENTER_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }

        break;
      // Red alliance right grid
      case 3:
        if (RobotContainer.robotState == RobotStates.HIGH_LEFT || RobotContainer.robotState == RobotStates.MID_LEFT) {
          targetXPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_RIGHT || RobotContainer.robotState == RobotStates.MID_RIGHT) {
          targetXPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_CENTER || RobotContainer.robotState == RobotStates.MID_CENTER) {
          targetXPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.RED_RIGHT_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }

        break;
      // Blue alliance substation
      case 4:
        if (RobotContainer.robotState == RobotStates.LEFT_SUBSTATION) {
          targetXPose = VisionConstants.BLUE_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.BLUE_SUBSTATION_POSE_Y + VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 180;

        } else if (RobotContainer.robotState == RobotStates.RIGHT_SUBSTATION) {
          targetXPose = VisionConstants.BLUE_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.BLUE_SUBSTATION_POSE_Y - VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 180;

        }

        break;
      // Red alliance substation
      case 5:
        if (RobotContainer.robotState == RobotStates.LEFT_SUBSTATION) {
          targetXPose = VisionConstants.RED_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.RED_SUBSTATION_POSE_Y + VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 180;

        } else if (RobotContainer.robotState == RobotStates.RIGHT_SUBSTATION) {
          targetXPose = VisionConstants.RED_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_X_OFFSET;
          targetYPose = VisionConstants.RED_SUBSTATION_POSE_Y - VisionConstants.SUBSTATION_Y_OFFSET;
          targetYawAngle = 180;

        }

        break;
      // Blue alliance left grid
      case 6:
        if (RobotContainer.robotState == RobotStates.HIGH_LEFT || RobotContainer.robotState == RobotStates.MID_LEFT) {
          targetXPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_RIGHT || RobotContainer.robotState == RobotStates.MID_RIGHT) {
          targetXPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_CENTER || RobotContainer.robotState == RobotStates.MID_CENTER) {
          targetXPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_LEFT_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }
        break;
      // Blue alliance center grid
      case 7:
        if (RobotContainer.robotState == RobotStates.HIGH_LEFT || RobotContainer.robotState == RobotStates.MID_LEFT) {
          targetXPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_RIGHT || RobotContainer.robotState == RobotStates.MID_RIGHT) {
          targetXPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_CENTER || RobotContainer.robotState == RobotStates.MID_CENTER) {
          targetXPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_CENTER_GRID_CUBE_POSE_Y;
          targetYawAngle = 0;
        }
        break;
      // Blue alliance right grid
      case 8:
        if (RobotContainer.robotState == RobotStates.HIGH_LEFT || RobotContainer.robotState == RobotStates.MID_LEFT) {
          targetXPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_Y + VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_RIGHT || RobotContainer.robotState == RobotStates.MID_RIGHT) {
          targetXPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_X + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
              + VisionConstants.GRID_OFFSET;
          targetYPose = VisionConstants.BLUE_RIGHT_GRID_CUBE_POSE_Y - VisionConstants.CONE_OFFSET;
          targetYawAngle = 0;

        } else if (RobotContainer.robotState == RobotStates.HIGH_CENTER || RobotContainer.robotState == RobotStates.MID_CENTER) {
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

    if (allianceColor == "Red") {
      driveSubsystem.resetOdometry(new Pose2d(LimelightSubsystem.getRedBotPoseX("limelight-two"),
          LimelightSubsystem.getRedBotPoseY("limelight-two"), GyroSubsystem.getYawAngle()));

    } else if (allianceColor == "Blue") {
      driveSubsystem.resetOdometry(new Pose2d(LimelightSubsystem.getBlueBotPoseX("limelight-two"),
          LimelightSubsystem.getBlueBotPoseY("limelight-two"), GyroSubsystem.getYawAngle()));

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      currentXPose = driveSubsystem.getPose2d().getX();
      currentYPose = driveSubsystem.getPose2d().getY();
      currentYawAngle = driveSubsystem.getPose2d().getRotation().getDegrees();

      xPercentAdjust = MathUtil.clamp(xPIDController.calculate(currentXPose), -1, 1);
      yPercentAdjust = MathUtil.clamp(yPIDController.calculate(currentYPose), -1, 1);
      yawPercentAdjust = MathUtil.clamp(yawPIDController.calculate(currentYawAngle), -1, 1);

      driveSubsystem.autoDrive(xPercentAdjust, yPercentAdjust, yawPercentAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.autoDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPIDController.atSetpoint() && yPIDController.atSetpoint() && yawPIDController.atSetpoint();
  }
}
