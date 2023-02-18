// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ScoringVisionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private String limelightHostName = "limelight-two";

  private int targetTagID;

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
    if (DriverStation.getAlliance() == Alliance.Red) {
      xTagDistance = LimelightSubsystem.getRedBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getRedBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getRedBotPoseYaw(limelightHostName);
    } else if (DriverStation.getAlliance() == Alliance.Blue) {
      xTagDistance = LimelightSubsystem.getBlueBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getBlueBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getBlueBotPoseYaw(limelightHostName);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.resetOdometry(new Pose2d(xTagDistance, yTagDistance, GyroSubsystem.getYawAngle()));
    targetTagID = (int) LimelightSubsystem.getAprilTagID(limelightHostName);

    if (DriverStation.getAlliance() == Alliance.Red) {
      switch (targetTagID) {
        // Left grid (from driver perspective) on red alliance
        case 1:
          
          break;
        // Middle grid (from driver perspective) on red alliance
        case 2:

          break;
        // Right grid (from driver perspective) on red alliance
        case 3:

          break;
      }
    } else if (DriverStation.getAlliance() == Alliance.Blue) {
      switch (targetTagID) {
        // Left grid (from driver perspective) on blue alliance
        case 6:

          break;
        // Middle grid (from driver perspective) on blue alliance
        case 7:

          break;
        // Right grid (from driver perspective) on blue alliance
        case 8:

          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
