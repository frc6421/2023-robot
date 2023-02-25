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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SubstationVisionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  // All distances are in meters \\
  private double tagID;
  private double xTagDistance;
  private double targetXDistance;
  private double xDistanceError;
  private double allowableXError = 0.03;
  private double xPercentAdjust;
  private double xP = 0.35; // TODO update with constant

  private double yTagDistance;
  private double targetYDistance;
  private double yDistanceError;
  private double allowableYError = 0.03;
  private double yPercentAdjust;
  private double yP = 0.35; // TODO update with constant

  private double yawTagAngle;
  private double targetYawAngle;
  private double yawAngleError;
  private double allowableYawError = 10;
  private double yawPercentAdjust;
  private double yawP = 0.01; // TODO update with constant

  private double feedForward = 0.09;

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
    LimelightSubsystem.setAprilTagPipeline(limelightHostName);
    LimelightSubsystem.setPipelineLEDControl(limelightHostName);
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

      targetXDistance = VisionConstants.RED_SUBSTATION_POSE_X - VisionConstants.SUBSTATION_OFFSET;
      targetYDistance = VisionConstants.RED_SUBSTATION_POSE_Y;
      targetYawAngle = 0;

      xDistanceError = xTagDistance - targetXDistance;
      yDistanceError = yTagDistance - targetYDistance;
      yawAngleError = yawTagAngle - targetYawAngle;

    } else if (DriverStation.getAlliance() == Alliance.Blue && tagID == 4) {
      // Set target distance based on field relative pose
      xTagDistance = LimelightSubsystem.getBlueBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getBlueBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getBlueBotPoseYaw(limelightHostName);

      targetXDistance = VisionConstants.BLUE_SUBSTATION_POSE_X - VisionConstants.SUBSTATION_OFFSET;
      targetYDistance = VisionConstants.BLUE_SUBSTATION_POSE_Y;
      targetYawAngle = 0;

      xDistanceError = targetXDistance - xTagDistance;
      yDistanceError = targetYDistance - yTagDistance;
      yawAngleError = targetYawAngle - yawTagAngle;
    }

    xPercentAdjust = (xDistanceError * xP) + (Math.signum(xDistanceError) * feedForward);
    yPercentAdjust = (yDistanceError * yP) + (Math.signum(yDistanceError) * feedForward);
    yawPercentAdjust = (yawAngleError * yawP) + (Math.signum(yawAngleError) * feedForward);

    xPercentAdjust = MathUtil.clamp(xPercentAdjust, -1, 1);
    yPercentAdjust = MathUtil.clamp(yPercentAdjust, -1, 1);
    yawPercentAdjust = MathUtil.clamp(yawPercentAdjust, -1, 1);

    driveSubsystem.autoDrive(-xPercentAdjust, -yPercentAdjust, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Entered Substation End");

    driveSubsystem.autoDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return Math.abs(xTagDistance - (VisionConstants.RED_SUBSTATION_POSE_X + VisionConstants.SUBSTATION_OFFSET)) <= allowableXError
          && Math.abs(yTagDistance - VisionConstants.RED_SUBSTATION_POSE_Y) <= allowableYError
          && Math.abs(yawTagAngle - targetYawAngle) <= allowableYawError;

    } else if (DriverStation.getAlliance() == Alliance.Blue) {
      return Math.abs(xTagDistance - (VisionConstants.BLUE_SUBSTATION_POSE_X - VisionConstants.SUBSTATION_OFFSET)) <= allowableXError
          && Math.abs(yTagDistance - VisionConstants.BLUE_SUBSTATION_POSE_Y) <= allowableYError
          && Math.abs(yawTagAngle - targetYawAngle) <= allowableYawError;

    } else {
      return true;
    }

  }
}
