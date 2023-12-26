// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DriveCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private DoubleSupplier leftYSupplier;
  private DoubleSupplier leftXSupplier;
  private DoubleSupplier rightXSupplier;

  private BooleanSupplier angleSupplierA;
  private BooleanSupplier angleSupplierB;
  private BooleanSupplier angleSupplierX;
  private BooleanSupplier angleSupplierY;

  private double xSpeedInput;
  private double ySpeedInput;
  private double rotationInput;

  private double currentAngle;
  private double targetAngle;

  private PIDController angleController;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX,
      BooleanSupplier angleA, BooleanSupplier angleB, BooleanSupplier angleX, BooleanSupplier angleY) {
    driveSubsystem = drive;
    leftYSupplier = leftY;
    leftXSupplier = leftX;
    rightXSupplier = rightX;

    angleSupplierA = angleA;
    angleSupplierB = angleB;
    angleSupplierX = angleX;
    angleSupplierY = angleY;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeedInput = MathUtil.applyDeadband(leftXSupplier.getAsDouble(), ModuleConstants.PERCENT_DEADBAND);
    ySpeedInput = MathUtil.applyDeadband(leftYSupplier.getAsDouble(), ModuleConstants.PERCENT_DEADBAND);
    rotationInput = MathUtil.applyDeadband(rightXSupplier.getAsDouble(), ModuleConstants.PERCENT_DEADBAND);

    double xSpeed;
    double ySpeed;
    double rotation;

    xSpeed = -1 * (xSpeedInput) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    ySpeed = -1 * (ySpeedInput) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;

    currentAngle = GyroSubsystem.getYawAngle().getDegrees();

    if (angleSupplierA.getAsBoolean()) {
      targetAngle = 180;

    } else if (angleSupplierB.getAsBoolean()) {
      targetAngle = 90;

    } else if (angleSupplierX.getAsBoolean()) {
      targetAngle = -90;

    } else if (angleSupplierY.getAsBoolean()) {
      targetAngle = 0;

    } else {
      targetAngle = currentAngle;

    }

    if (targetAngle != currentAngle) {
      rotation = angleController.calculate(GyroSubsystem.getYawAngle().getDegrees(), targetAngle);
    } else {
      rotation = -1 * Math.signum(rotationInput) * (rotationInput * rotationInput)
          * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    rotation = MathUtil.clamp(rotation, -2 * Math.PI, 2 * Math.PI);

    // Sets chassis speeds
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation,
        GyroSubsystem.getYawAngle());

    // Sets field relative speeds to the swerve module states
    var swerveModuleStates = driveSubsystem.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    // X-lock wheels to prevent pushing and sets the speed of each module to 0
    if (chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 &&
        chassisSpeeds.omegaRadiansPerSecond == 0) {
      swerveModuleStates[0].angle = Rotation2d.fromDegrees(45);
      swerveModuleStates[0].speedMetersPerSecond = 0;
      swerveModuleStates[1].angle = Rotation2d.fromDegrees(-45);
      swerveModuleStates[1].speedMetersPerSecond = 0;
      swerveModuleStates[2].angle = Rotation2d.fromDegrees(-45);
      swerveModuleStates[2].speedMetersPerSecond = 0;
      swerveModuleStates[3].angle = Rotation2d.fromDegrees(45);
      swerveModuleStates[3].speedMetersPerSecond = 0;
    }

    // Sets the swerve modules to their desired states using optimization method
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
}
