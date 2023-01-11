// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class ModuleSubsystem extends SubsystemBase {
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX steerMotor;

  private final CANCoder steerEncoder;

  private final PIDController drivePIDController = new PIDController(ModuleConstants.MODULE_DRIVE_P, ModuleConstants.MODULE_DRIVE_I, ModuleConstants.MODULE_DRIVE_D);

  private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
    ModuleConstants.MODULE_STEER_P,
    ModuleConstants.MODULE_STEER_I,
    ModuleConstants.MODULE_STEER_D,
    new TrapezoidProfile.Constraints(ModuleConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 
    ModuleConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

  /** Creates a new ModuleSubsystem. */
  public ModuleSubsystem(int driveMotorID, int steerMotorID, int steerEncoderID, double angleOffset) {
    driveMotor = new WPI_TalonFX(driveMotorID);
    steerMotor = new WPI_TalonFX(steerMotorID);

    steerEncoder = new WPI_CANCoder(steerEncoderID);

    driveMotor.configFactoryDefault();
    steerMotor.configFactoryDefault();

    //TODO check to see if this is correct
    //steerMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180, steerEncoderID);

    driveMotor.setInverted(true);
    steerMotor.setInverted(false);

    driveMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setNeutralMode(NeutralMode.Brake);

    steerEncoder.configFactoryDefault();

    steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    steerEncoder.configSensorDirection(true); // Clockwise
    steerEncoder.configMagnetOffset(Math.toDegrees(angleOffset)); //TODO check method

    driveMotor.configNeutralDeadband(0.02); //TODO determine experimentally
    steerMotor.configNeutralDeadband(0.02); 
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
