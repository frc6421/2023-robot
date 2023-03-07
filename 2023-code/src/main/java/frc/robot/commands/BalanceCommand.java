// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChargeStationConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class BalanceCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;

  private final TrapezoidProfile.Constraints constraints;
  private final ProfiledPIDController profiledPIDController;
  private double currentGyroAngle; 
  private double gyroPValue = .011;
  private double allowableAngleError = 1.5;
  private double angleAdjust;
  private SwerveModuleState[] states = new SwerveModuleState[4];

  /** Creates a new BalanceCommand. */
  public BalanceCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    addRequirements(drive);
    constraints = new TrapezoidProfile.Constraints(ChargeStationConstants.CHARGE_MAX_VELOCITY, ChargeStationConstants.CHARGE_MAX_ACCELERATION);
    profiledPIDController = new ProfiledPIDController(gyroPValue, 0, 0, constraints);
    profiledPIDController.setTolerance(allowableAngleError);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    profiledPIDController.setGoal(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentGyroAngle = GyroSubsystem.getPitchAngleDouble();

    angleAdjust = MathUtil.clamp(profiledPIDController.calculate(currentGyroAngle), -1, 1);
    angleAdjust *= DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    for (int index = 0; index < 4; index++) {
      states[index] = new SwerveModuleState(angleAdjust, new Rotation2d(0));
    }
    driveSubsystem.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //states = driveSubsystem.setWheelsIn();
    driveSubsystem.setModuleStates(states);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
