// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;

public class VisionCommand extends CommandBase {
  private String allianceColor = DriverStation.getAlliance().name();

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

  private final TrapezoidProfile.Constraints translationConstraints;
  private final TrapezoidProfile.Constraints rotationConstraints;

  private final ProfiledPIDController xProfiledPIDController;
  private final ProfiledPIDController yProfiledPIDController;
  private final ProfiledPIDController yawProfiledPIDController;

  /** Creates a new VisionCommand. */
  public VisionCommand() {

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
