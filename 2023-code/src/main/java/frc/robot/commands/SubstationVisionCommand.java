// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SubstationVisionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  // ALL DISTANCES ARE IN METERS \\
  private double tagID;
  private double xTagDistance;
  private double targetXDistance = 1;
  private double allowableXError = 0.01;
  private double xP = 4; // TODO update with constant

  private double yTagDistance;
  private double targetYDistance = 0;
  private double allowableYError = 0.01;
  private double yP = 4; // TODO update with constant

  private double yawTagAngle;
  private double targetYawAngle = 0;
  private double allowableYawError = 1;
  private double yawP = 1; // TODO update with constant

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
    if (DriverStation.getAlliance() == Alliance.Red && tagID == 5) {
      // Set target distance based on field relative pose
      xTagDistance = LimelightSubsystem.getRedBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getRedBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getRedBotPoseYaw(limelightHostName);

      double xDistanceError = xTagDistance - targetXDistance;
      double yDistanceError = yTagDistance - targetYDistance;
      double yawAngleError = yawTagAngle - targetYawAngle;

      double xPercentAdjust = (xDistanceError * xP);
      double yPercentAdjust = (yDistanceError * yP);
      double yawPercentAdjust = (yawAngleError * yawP);

      driveSubsystem.visionDrive(xPercentAdjust, yPercentAdjust, yawPercentAdjust);

    } else if (DriverStation.getAlliance() == Alliance.Blue && tagID == 4) {
      // Set target distance based on field relative pose
      xTagDistance = LimelightSubsystem.getBlueBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getBlueBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getBlueBotPoseYaw(limelightHostName);

      double xDistanceError = xTagDistance - targetXDistance;
      double yDistanceError = yTagDistance - targetYDistance;
      double yawAngleError = yawTagAngle - targetYawAngle;

      double xPercentAdjust = (xDistanceError * xP);
      double yPercentAdjust = (yDistanceError * yP);
      double yawPercentAdjust = (yawAngleError * yawP);

      driveSubsystem.visionDrive(xPercentAdjust, yPercentAdjust, yawPercentAdjust);
    }
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

    return Math.abs(xTagDistance) <= allowableXError && Math.abs(yTagDistance) <= allowableYError && Math.abs(yawTagAngle) <= allowableYawError;
  }
}
