// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule{
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX steerMotor;

  private final WPI_CANCoder steerEncoder;

  private final SimpleMotorFeedforward feedforward;

  // TODO Maybe used for testing
  private double referenceVoltage = 0;
  private double referenceAngle = 0;

  /** Creates a new ModuleSubsystem. */
  public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderID, double angleOffset) {
    driveMotor = new WPI_TalonFX(driveMotorID, ModuleConstants.CANIVORE_NAME);
    steerMotor = new WPI_TalonFX(steerMotorID, ModuleConstants.CANIVORE_NAME);

    steerEncoder = new WPI_CANCoder(steerEncoderID, ModuleConstants.CANIVORE_NAME);

    feedforward = new SimpleMotorFeedforward(DriveConstants.S_VOLTS, DriveConstants.V_VOLT_SECONDS_PER_METER, DriveConstants.A_VOLT_SECONDS_SQUARED_PER_METER);

    driveMotor.configFactoryDefault();
    steerMotor.configFactoryDefault();

    driveMotor.setInverted(true);
    steerMotor.setInverted(true);

    driveMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setNeutralMode(NeutralMode.Coast);

    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // For velocity control
    driveMotor.config_kP(0, ModuleConstants.MODULE_DRIVE_P);
    driveMotor.config_kI(0, ModuleConstants.MODULE_DRIVE_I);
    driveMotor.config_kD(0, ModuleConstants.MODULE_DRIVE_D);

    // For position control
    steerMotor.config_kP(0, ModuleConstants.MODULE_STEER_P);
    steerMotor.config_kI(0, ModuleConstants.MODULE_STEER_I);
    steerMotor.config_kD(0, ModuleConstants.MODULE_STEER_D);

    steerMotor.configAllowableClosedloopError(0, 0.5 * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE);

    steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    steerEncoder.configSensorDirection(false); // Counter Clockwise
    steerEncoder.configMagnetOffset(angleOffset);
    steerEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    driveMotor.configNeutralDeadband(ModuleConstants.PERCENT_DEADBAND); // TODO determine experimentally
    steerMotor.configNeutralDeadband(ModuleConstants.PERCENT_DEADBAND);

    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1)); // TODO verify current
    steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1));

    Timer.delay(1.0);
    setSteerMotorToAbsolute();
  }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }

  // DRIVE MOTOR METHODS \\

  /**
   * Returns the drive motor velocity using encoder counts
   * Not currently used
   * 
   * @return drive motor velocity in meters per second
   */
  public double getDriveMotorVelocity() {
    return ((driveMotor.getSelectedSensorVelocity() / DriveConstants.GEAR_RATIO_MOTOR_TO_WHEEL) *
        (10.0 / DriveConstants.COUNTS_PER_ROTATION) * DriveConstants.WHEEL_CIRCUMFERENCE);
  }

  /**
   * Returns the motor voltage applied to the drive motor in volts
   * Not currently used
   * 
   * @return applied motor voltage in volts
   */
  public double getDriveMotorVoltage() {
    return driveMotor.getMotorOutputVoltage();
  }

  /**
   * Returns the drive motor distance using calculation for distance per encoder
   * count
   * 
   * @return drive motor distance in meters
   */
  public double getDriveMotorDistance() {
    return driveMotor.getSelectedSensorPosition() * DriveConstants.DISTANCE_PER_ENCODER_COUNT;
  }

  /**
   * Converts the drive encoder velocity from counts per 100 ms to meters per
   * second
   * 
   * @return drive encoder velocity in meters per second
   */
  public double getDriveMotorEncoderVelocity() {
    return driveMotor.getSelectedSensorVelocity() * 10 * DriveConstants.DISTANCE_PER_ENCODER_COUNT;
  }

  // STEER MOTOR METHODS \\

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
   * Gets the steer motor's current angle in degrees
   * @return steer motor's angle in degrees
   */
  public double getSteerMotorEncoderAngle() {
    return steerMotor.getSelectedSensorPosition() / DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;
  }

  // CANCODER METHODS \\

  /**
   * Returns the angle measured on the CANcoder (steering encoder)
   * 
   * @return wheel angle in radians
   */
  public double getCANcoderRadians() {
    return Math.toRadians(steerEncoder.getAbsolutePosition());
  }

  /**
   * Gets the module position based on distance traveled in meters for drive motor
   * and degrees for steering motor
   * Used for odometry
   * 
   * @return current SwerveModulePosition
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDriveMotorDistance(), new Rotation2d(getCANcoderRadians()));
  }

  /**
   * Optimizes the swerve module outputs and applies the drive percent output and steer position
   * 
   * @param desiredState
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state to avoid spinning modules more than 90 degrees
    SwerveModuleState state = customOptimize(desiredState, new Rotation2d(Math.toRadians(getSteerMotorEncoderAngle())));

    // Calculate percent of max drive velocity
    double driveOutput = state.speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;

    // Calculate steer motor output
    double steerPositionOutput = state.angle.getDegrees() * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;

    // Apply PID outputs
    driveMotor.set(ControlMode.PercentOutput, driveOutput);
    steerMotor.set(ControlMode.Position, steerPositionOutput);
  }

  /**
   * Optimizes the swerve module outputs and applies the drive percent output and steer position
   * Closed loop output
   * 
   * @param desiredState
   */
  public void autoSetDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state to avoid spinning modules more than 90 degrees
    SwerveModuleState state = customOptimize(desiredState, new Rotation2d(Math.toRadians(getSteerMotorEncoderAngle())));

    // Calculate percent of max drive velocity
    double driveOutput = state.speedMetersPerSecond / DriveConstants.DISTANCE_PER_ENCODER_COUNT / 10;

    // Calculate steer motor output
    double steerPositionOutput = state.angle.getDegrees() * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;

    // Apply PID outputs
    driveMotor.set(ControlMode.Velocity, driveOutput, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
    steerMotor.set(ControlMode.Position, steerPositionOutput);
  }


  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    steerEncoder.setPosition(0);
  }

  /**
   * From team 364
   * 
   * Minimize the change in heading the desired swerve module state would require
   * by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to
   * include placing
   * in appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState customOptimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * From team 364
   * 
   * @param initialAngle Current Angle
   * @param targetAngle  Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double initialAngle, double targetAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = initialAngle % 360;
    if (lowerOffset >= 0) {
      lowerBound = initialAngle - lowerOffset;
      upperBound = initialAngle + (360 - lowerOffset);
    } else {
      upperBound = initialAngle - lowerOffset;
      lowerBound = initialAngle - (360 + lowerOffset);
    }
    while (targetAngle < lowerBound) {
      targetAngle += 360;
    }
    while (targetAngle > upperBound) {
      targetAngle -= 360;
    }
    if (targetAngle - initialAngle > 180) {
      targetAngle -= 360;
    } else if (targetAngle - initialAngle < -180) {
      targetAngle += 360;
    }
    return targetAngle;
  }
}
