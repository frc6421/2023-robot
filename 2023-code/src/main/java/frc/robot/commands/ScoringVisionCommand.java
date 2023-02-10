// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ScoringVisionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private String limelightHostName = "limelight-two";
  private double targetTagID;
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
    xTagDistance = LimelightSubsystem.getBotPoseX(limelightHostName);
    yTagDistance = LimelightSubsystem.getBotPoseY(limelightHostName);
    yawTagAngle = LimelightSubsystem.getBotPoseYaw(limelightHostName);
    
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
