// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class TurnToAngleCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private DoubleSupplier xValue;
  private DoubleSupplier yValue;

  private double rotation;
  private double targetAngle;

  private PIDController angleController;

  /** Creates a new TurnToAngleCommand. */
  public TurnToAngleCommand(DoubleSupplier x, DoubleSupplier y, DriveSubsystem drive) {
    driveSubsystem = drive;
    xValue = x;
    yValue = y;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    // PID controller for the rotation of the robot
    angleController = new PIDController(DriveConstants.ANGLE_CONTROLLER_KP, 0, 0);
    angleController.enableContinuousInput(-180, 180);
    angleController.setTolerance(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = new Rotation2d(xValue.getAsDouble(), yValue.getAsDouble()).getDegrees();
    //angleController.setSetpoint(targetAngle);
    SmartDashboard.putNumber("Target Angle", targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetAngle = new Rotation2d(xValue.getAsDouble(), yValue.getAsDouble()).getDegrees();
    SmartDashboard.putNumber("Target Angle", targetAngle);
    SmartDashboard.putNumber("x", xValue.getAsDouble());
    SmartDashboard.putNumber("y", yValue.getAsDouble());
    // rotation = angleController.calculate(GyroSubsystem.getYawAngle().getDegrees());

    // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotation,
    //     GyroSubsystem.getYawAngle());

    // // Sets field relative speeds to the swerve module states
    // var swerveModuleStates = driveSubsystem.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    // driveSubsystem.setModuleStates(swerveModuleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleController.atSetpoint();
  }
}
