// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ScoringVisionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private String limelightHostName = "limelight-two";
  private int targetTagID;
  private double xTagDistance;
  private double yTagDistance;
  private double yawTagAngle;
  private double xRetroAngle;

  /** Creates a new ScoringVisionCommand. */
  public ScoringVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      xTagDistance = LimelightSubsystem.getRedBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getRedBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getRedBotPoseYaw(limelightHostName);
    } else if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      xTagDistance = LimelightSubsystem.getBlueBotPoseX(limelightHostName);
      yTagDistance = LimelightSubsystem.getBlueBotPoseY(limelightHostName);
      yawTagAngle = LimelightSubsystem.getBlueBotPoseYaw(limelightHostName);
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetTagID = (int)LimelightSubsystem.getAprilTagID(limelightHostName);

    switch(targetTagID) {
      // Left grid (from driver perspective) on red alliance
      case 1:

      break;
      // Middle grid (from driver perspective) on red alliance
      case 2:

      break;
      // Right grid (from driver perspective) on red alliance
      case 3:

      break;
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
