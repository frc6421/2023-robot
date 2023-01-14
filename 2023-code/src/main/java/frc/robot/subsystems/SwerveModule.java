// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX steerMotor;

  private final CANCoder steerEncoder;

  private final double angleOffset;

  private final PIDController drivePIDController = new PIDController(ModuleConstants.MODULE_DRIVE_P, ModuleConstants.MODULE_DRIVE_I, ModuleConstants.MODULE_DRIVE_D);

  private final ProfiledPIDController steeringPIDController = new ProfiledPIDController(
    ModuleConstants.MODULE_STEER_P,
    ModuleConstants.MODULE_STEER_I,
    ModuleConstants.MODULE_STEER_D,
    new TrapezoidProfile.Constraints(DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 
    DriveConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

  private SimpleMotorFeedforward driveFeedforward;
  private SimpleMotorFeedforward steerFeedforward;

  private double referenceVoltage = 0;
  private double referenceAngle = 0;

  /** Creates a new ModuleSubsystem. */
  public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderID, double angleOffset) {
    driveMotor = new WPI_TalonFX(driveMotorID);
    steerMotor = new WPI_TalonFX(steerMotorID);

    steerEncoder = new WPI_CANCoder(steerEncoderID);

    this.angleOffset = angleOffset;

    driveMotor.configFactoryDefault();
    steerMotor.configFactoryDefault();

    //TODO check to see if this is correct
    //steerMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180, steerEncoderID);

    driveMotor.setInverted(true);
    steerMotor.setInverted(false);

    driveMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setNeutralMode(NeutralMode.Brake);

    driveMotor.config_kP(0, ModuleConstants.MODULE_DRIVE_P);
    driveMotor.config_kI(0, ModuleConstants.MODULE_DRIVE_I);
    driveMotor.config_kD(0, ModuleConstants.MODULE_DRIVE_D);

    steerMotor.config_kP(0, ModuleConstants.MODULE_STEER_P);
    steerMotor.config_kI(0, ModuleConstants.MODULE_STEER_I);
    steerMotor.config_kD(0, ModuleConstants.MODULE_STEER_D);

    steerMotor.configAllowableClosedloopError(0, 0.5 * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE);

    steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    steerEncoder.configSensorDirection(true); // Clockwise
    steerEncoder.configMagnetOffset(angleOffset); //TODO check method, may not be correct
    steerEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    driveMotor.configNeutralDeadband(0.02); //TODO determine experimentally
    steerMotor.configNeutralDeadband(0.02);

    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 80, 0, 1)); //TODO verify current limit for drive and steer motors
    steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 0, 1));

    setSteerMotorToAbsolute();

    steeringPIDController.enableContinuousInput(0, 2 * Math.PI);

    driveFeedforward = new SimpleMotorFeedforward(
      DriveConstants.S_VOLTS,
      DriveConstants.V_VOLT_SECONDS_PER_METER,
      DriveConstants.A_VOLT_SECONDS_SQUARED_PER_METER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the target voltage for the drivetrain by converting m/s to volts
   * 
   * @param targetVelocity (double) target velocity in meters per second
   */
  public void setReferenceVoltage(double targetVelocity) {
    referenceVoltage = targetVelocity * (DriveConstants.MAX_VOLTAGE / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    driveMotor.setVoltage(drivePIDController.calculate(getDriveMotorVoltage(), referenceVoltage));
  }

  public double getReferenceVoltage() {
    return referenceVoltage;
  }

  /**
   * Returns the drive motor velocity using encoder counts
   * 
   * @return drive motor velocity in meters per second
   */
  public double getDriveMotorVelocity() {
    return ((driveMotor.getSelectedSensorVelocity() / DriveConstants.GEAR_RATIO_MOTOR_TO_WHEEL) * 
      (10.0 / DriveConstants.COUNTS_PER_ROTATION) * DriveConstants.WHEEL_CIRCUMFERENCE);
  }

  /**
   * Returns the motor voltage applied to the drive motor in volts
   * 
   * @return applied motor voltage in volts
   */
  public double getDriveMotorVoltage() {
    return driveMotor.getMotorOutputVoltage();
  }

  /**
   * Set the target angle of the module in radians
   * 
   * @param targetAngle (double) target angle in degrees
   */
  public void setReferenceAngle(double targetAngle) {
    referenceAngle = Math.toRadians(targetAngle);

    steerMotor.setVoltage(steeringPIDController.calculate(getReferenceAngleRadians(), referenceAngle));
  }

  /**
   * Returns the angle measured on the CANcoder (steering encoder)
   * 
   * @return wheel angle in radians
   */
  public double getReferenceAngleRadians() {
    return Math.toRadians(steerEncoder.getAbsolutePosition());
  }

  /**
   * Returns the drive motor distance using calculation for distance per encoder count
   * 
   * @return drive motor distance in meters
   */
  public double getDriveMotorDistance() {
    return driveMotor.getSelectedSensorPosition() * DriveConstants.DISTANCE_PER_ENCODER_COUNT;
  }

  /**
   * Converts the drive encoder velocity from counts per 100 ms to meters per second
   * 
   * @return drive encoder velocity in meters per second
   */
  public double getDriveEncoderVelocity() {
    return driveMotor.getSelectedSensorVelocity() * 10 * DriveConstants.DISTANCE_PER_ENCODER_COUNT;
  }

  /**
   * Sets the of the steer motor encoder to the value of the CANcoder
   * 
   */
  public void setSteerMotorToAbsolute() {
    double currentAngle = steerEncoder.getAbsolutePosition();
    double absolutePosition = currentAngle * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;
    steerMotor.setSelectedSensorPosition(absolutePosition);
  }

  /**
   * Gets the module position based on distance traveled in meters for drive motor and degrees for steering motor
   * Used for odometry
   * 
   * @return current SwerveModulePosition
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDriveMotorDistance(), new Rotation2d(getReferenceAngleRadians()));
  }

  /**
   * Optimizes the swerve module outputs and applies the drive and steer PIDs
   * 
   * @param desiredState 
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state to avoid spinning modules more than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getReferenceAngleRadians()));

    // Calculate percent of max drive velocity
    double driveOutput = state.speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;

    // Calculate steer motor output from turning PID controller
    double steerPositionOutput = state.angle.getDegrees() * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;

    // Apply PID outputs
    driveMotor.set(ControlMode.PercentOutput, driveOutput);
    steerMotor.set(ControlMode.Position, steerPositionOutput);
  }

  public double getSteerMotorEncoderAngle() {
    return steerMotor.getSelectedSensorPosition() / DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;
  }

  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    steerEncoder.setPosition(0);
  }

}
