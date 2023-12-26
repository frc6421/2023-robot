// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class TurnToAngleCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private double xValue;
  private double yValue;

  private double rotation;
  private double targetAngle;

  private PIDController angleController;

  /** Creates a new TurnToAngleCommand. */
  public TurnToAngleCommand(DoubleSupplier x, DoubleSupplier y, DriveSubsystem drive) {
    driveSubsystem = drive;
    xSupplier = x;
    ySupplier = y;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    // PID controller for the rotation of the robot
    angleController = new PIDController(DriveConstants.ANGLE_CONTROLLER_KP, 0, 0);
    angleController.enableContinuousInput(-180, 180);
    angleController.setTolerance(2);

    // Sendable to Dashboard.  Only if not attached to FMS.
    if (!DriverStation.isFMSAttached()) {
      SmartDashboard.putData("Turn to Angle", this);
      SmartDashboard.putData("Turn PID", angleController);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Check if joystick is being used.
    if ( Math.abs(xValue) <= 0.5 && Math.abs(yValue) <= 0.5) {
      // If not used, keep angle of robot at current angle
      targetAngle = GyroSubsystem.getYawAngle().getDegrees();
    }
    else {
      // If used, get angle based on joystick position
      targetAngle = new Rotation2d(xValue, yValue).getDegrees();
    }

    // xValue = MathUtil.applyDeadband(xSupplier.getAsDouble(), ModuleConstants.PERCENT_DEADBAND);
    // yValue = MathUtil.applyDeadband(ySupplier.getAsDouble(), ModuleConstants.PERCENT_DEADBAND);

    targetAngle = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).getDegrees();
    
    rotation = angleController.calculate(GyroSubsystem.getYawAngle().getDegrees(), targetAngle);

    rotation = MathUtil.clamp(rotation, -2 * Math.PI, 2 * Math.PI);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotation,
        GyroSubsystem.getYawAngle());

    // Sets field relative speeds to the swerve module states
    var swerveModuleStates = driveSubsystem.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    driveSubsystem.setModuleStates(swerveModuleStates);
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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Command");
    builder.addStringProperty(".name", this::getName, null);
    builder.addBooleanProperty(
        "running",
        this::isScheduled,
        value -> {
          if (value) {
            if (!isScheduled()) {
              schedule();
            }
          } else {
            if (isScheduled()) {
              cancel();
            }
          }
        });
    builder.addDoubleProperty("X Joystick Value", () -> xValue, null);
    builder.addDoubleProperty("Y Joystick Value", () -> yValue, null);
    builder.addDoubleProperty("Target Angle", () -> targetAngle, null);
  }
}
