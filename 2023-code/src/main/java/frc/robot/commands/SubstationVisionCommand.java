// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SubstationVisionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  // ALL DISTANCES ARE IN METERS \\
  private double tagID;
  private double xTagDistance;
  private double targetXDistance = 1;
  private double xDistanceError;
  private double allowableXError = 0.01;
  private double xPercentAdjust;
  private double xP = 0.3; // TODO update with constant

  private double yTagDistance;
  private double targetYDistance = 0;
  private double yDistanceError;
  private double allowableYError = 0.01;
  private double yPercentAdjust;
  private double yP = 0.6; // TODO update with constant

  private double yawTagAngle;
  private double targetYawAngle = 0;
  private double yawAngleError;
  private double allowableYawError = 1;
  private double yawPercentAdjust;
  private double yawP = 0.01; // TODO update with constant

  private double feedForward = 0.095;

  private String limelightHostName = "limelight-two";

  /** Creates a new SubstationVisionCommand. */
  public SubstationVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tagID = LimelightSubsystem.getAprilTagID(limelightHostName);

    if (DriverStation.getAlliance() == Alliance.Red && tagID == 5) {
      // Set target distance based on field relative pose
      xTagDistance = LimelightSubsystem.getRedBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getRedBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getRedBotPoseYaw(limelightHostName);

      xDistanceError = xTagDistance - targetXDistance;
      yDistanceError = yTagDistance - targetYDistance;
      yawAngleError = yawTagAngle - targetYawAngle;

    } else if (DriverStation.getAlliance() == Alliance.Blue && tagID == 4) {
      // Set target distance based on field relative pose
      xTagDistance = LimelightSubsystem.getBlueBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getBlueBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getBlueBotPoseYaw(limelightHostName);

      targetXDistance = VisionConstants.BLUE_SUBSTATION_X_POSE - 1;
      targetYDistance = VisionConstants.BLUE_SUBSTATION_Y_POSE;
      targetYawAngle = 0;

      xDistanceError = targetXDistance - xTagDistance;
      yDistanceError = targetYDistance - yTagDistance;
      yawAngleError = targetYawAngle - yawTagAngle;
    }

    xPercentAdjust = (xDistanceError * xP) + feedForward;
    yPercentAdjust = (yDistanceError * yP) + feedForward;
    yawPercentAdjust = (yawAngleError * yawP) + feedForward;

    xPercentAdjust = MathUtil.clamp(xPercentAdjust, -1, 1);
    yPercentAdjust = MathUtil.clamp(yPercentAdjust, -1, 1);
    yawPercentAdjust = MathUtil.clamp(yawPercentAdjust, -1, 1);

    driveSubsystem.visionDrive(0, 0, -yawPercentAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.visionDrive(0, 0, 0);
    // set arm to substation height
    // delay
    // activate gripper
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      xTagDistance = LimelightSubsystem.getRedBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getRedBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getRedBotPoseYaw(limelightHostName);
    } else if (DriverStation.getAlliance() == Alliance.Blue) {
      xTagDistance = LimelightSubsystem.getBlueBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getBlueBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getBlueBotPoseYaw(limelightHostName);
    }

    return Math.abs(xTagDistance) <= allowableXError && Math.abs(yTagDistance) <= allowableYError
        && Math.abs(yawTagAngle) <= allowableYawError;
  }
}
