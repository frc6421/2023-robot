// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
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

  private double allowableXError;
  private double allowableYError;
  private double allowableYawError;

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

    translationConstraints = new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND, AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    // TODO convert degrees to radians
    rotationConstraints = new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC, AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC);

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
    tagID = (int)LimelightSubsystem.getAprilTagID(limelightHostName);

    if(allianceColor == "Red") {
      currentXPose = LimelightSubsystem.getRedBotPoseX(limelightHostName);
      currentYPose = LimelightSubsystem.getRedBotPoseY(limelightHostName);
      currentYawAngle = LimelightSubsystem.getRedBotPoseYaw(limelightHostName);
    } else if(allianceColor == "Blue") {
      currentXPose = LimelightSubsystem.getBlueBotPoseX(limelightHostName);
      currentYPose = LimelightSubsystem.getBlueBotPoseY(limelightHostName);
      currentYawAngle = LimelightSubsystem.getBlueBotPoseYaw(limelightHostName);
    }

    switch(tagID) {
      case 1:

      break;
      case 2:

      break;
      case 3:

      break;
      case 4:

      break;
      case 5:

      break;
      case 6:

      break;
      case 7:

      break;
      case 8:

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

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
