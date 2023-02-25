// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SubstationGamePieceVisionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private String limelightHostName = "limelight-two";

  private double currentYAngle;
  private double targetYAngle = 19.5; //Constant angle for the piece to be at the right grabber position
  private double yAngleError;
  private double allowableYAngleError = 0.5;
  private double yPercentAdjust;
  private double yP = 0.3;

  private double currentXAngle;
  private double targetXAngle = 0;
  private double xAngleError;
  private double allowableXAngleError = 0.5;
  private double xPercentAdjust;
  private double xP = 0.3;

  private double feedForward = 0.09;

  /** Creates a new SubstationGamePieceVisionCommand. */
  public SubstationGamePieceVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_YELLOW) {
      BlinkinSubsystem.blinkinRedSet();
      LimelightSubsystem.setConePipeline(limelightHostName);
      LimelightSubsystem.setPipelineLEDControl(limelightHostName);
    } else if(BlinkinSubsystem.getBlinkinColor() == BlinkinConstants.BLINKIN_VIOLET) {
      BlinkinSubsystem.blinkinBlueSet();
      LimelightSubsystem.setCubePipeline(limelightHostName);
      LimelightSubsystem.setPipelineLEDControl(limelightHostName);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LimelightSubsystem.getPipelineID(limelightHostName) == 2 || LimelightSubsystem.getPipelineID(limelightHostName) == 3){
      currentXAngle = LimelightSubsystem.getX(limelightHostName);
      xAngleError = targetXAngle - currentXAngle;

      currentYAngle = LimelightSubsystem.getY(limelightHostName);
      yAngleError = targetYAngle - currentYAngle;

      xPercentAdjust = (xAngleError * xP) + (Math.signum(xAngleError) * feedForward);
      yPercentAdjust = (yAngleError * yP) + (Math.signum(yAngleError) * feedForward);

      xPercentAdjust = MathUtil.clamp(xPercentAdjust, -1, 1);
      yPercentAdjust = MathUtil.clamp(yPercentAdjust, -1, 1);

      driveSubsystem.autoDrive(-yPercentAdjust, -xPercentAdjust, 0);
    } else {
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.autoDrive(0, 0, 0);
    if(interrupted) {
      BlinkinSubsystem.blinkinConfettiSet();
    } else {
      BlinkinSubsystem.blinkinGreenSet();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(xAngleError) < allowableXAngleError && Math.abs(yAngleError) < allowableYAngleError;
  }
}
