// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final AHRS navx;

  private final SwerveDriveKinematics swerveKinematics;

  private final SwerveDriveOdometry odometry;
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontLeft = new SwerveModule(ModuleConstants.FRONT_LEFT_MODULE_DRIVE_CAN_ID, ModuleConstants.FRONT_LEFT_MODULE_STEER_CAN_ID, ModuleConstants.FRONT_LEFT_MODULE_ENCODER_CAN_ID, ModuleConstants.FRONT_LEFT_MODULE_ANGLE_OFFSET);
    frontRight = new SwerveModule(ModuleConstants.FRONT_RIGHT_MODULE_DRIVE_CAN_ID, ModuleConstants.FRONT_RIGHT_MODULE_STEER_CAN_ID, ModuleConstants.FRONT_RIGHT_MODULE_ENCODER_CAN_ID, ModuleConstants.FRONT_RIGHT_MODULE_ANGLE_OFFSET);
    backLeft = new SwerveModule(ModuleConstants.BACK_LEFT_MODULE_DRIVE_CAN_ID, ModuleConstants.BACK_LEFT_MODULE_STEER_CAN_ID, ModuleConstants.BACK_LEFT_MODULE_ENCODER_CAN_ID, ModuleConstants.BACK_LEFT_MODULE_ANGLE_OFFSET);
    backRight = new SwerveModule(ModuleConstants.BACK_RIGHT_MODULE_DRIVE_CAN_ID, ModuleConstants.BACK_RIGHT_MODULE_STEER_CAN_ID, ModuleConstants.BACK_RIGHT_MODULE_ENCODER_CAN_ID, ModuleConstants.BACK_RIGHT_MODULE_ANGLE_OFFSET);

    navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    
    swerveKinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    odometry = new SwerveDriveOdometry(swerveKinematics, getGyroRotation(), new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()});
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getGyroRotation(), new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()});

      SmartDashboard.putNumber("gyro", getGyroRotation().getDegrees());
  }

  // GYRO METHODS \\

  /**
   * Get gyro rotation using fused headings if available, standard rotation if unavailable
   * 
   * @return yaw rotation in degrees
   */
  public Rotation2d getGyroRotation() {
    if (navx.isMagnetometerCalibrated()) {
      return Rotation2d.fromDegrees(navx.getFusedHeading());
    }
    // Invert gyro angle so counterclockwise is positive
    return Rotation2d.fromDegrees(360 - navx.getAngle());
  }

  public void zeroGyro() {
    navx.zeroYaw();
  }

  /**
   * Get rate of gyro rotation in degrees per second
   * 
   * @return turn rate in degrees per second
   */
  public double getGyroRate() {
    return -navx.getRate();
  }

  // ODOMETRY METHODS \\

  /**
   * Returns estimated current robot pose in meters
   * 
   * @return current robot pose2d in meters
   */
  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets odometry to the given pose value
   * 
   * @param pose to use for reset
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroRotation(), new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()}, pose);
  }

  /**
   * Sets the swerve module states
   * 
   * @param desiredStates the desired swerve module states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Ensures all wheels obey max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    // Sets the swerve modules to their desired states using optimization method
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  //TODO not currently used
  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

   /**
   * Sets up our drive method
   * 
   * @param xSpeedInput value from -1.0 to 1.0 to convert to x-direction meters per second
   * @param ySpeedInput value from -1.0 to 1.0 to convert to y-direction meters per second
   * @param rotationInput value from -1.0 to 1.0 to convert to rotational speed in radians per second
   * @param magnitude value from 0-1 returned by the trigger to set the magnitude of x and y speeds (not rotational)
   */
  public void drive(double xSpeedInput, double ySpeedInput, double rotationInput, double magnitude) {
    // Set speed as a percentage of our max velocity
    double xSpeed = Math.signum(xSpeedInput) * magnitude * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    double ySpeed = Math.signum(ySpeedInput) * magnitude * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    double rotation = rotationInput * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // Sets field relative speeds
    var swerveModuleStates = 
      swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, getGyroRotation()));
      // Ensures all wheels obey max speed
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
      // Sets the swerve modules to their desired states using optimization method
      frontLeft.setDesiredState(swerveModuleStates[0]);
      frontRight.setDesiredState(swerveModuleStates[1]);
      backLeft.setDesiredState(swerveModuleStates[2]);
      backRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("FrontLeftDriveVelocity", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FrontRightDriveVelocity", swerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("BackLeftDriveVelocity", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("BackRightDriveVelocity", swerveModuleStates[3].speedMetersPerSecond);

    SmartDashboard.putNumber("FrontLeftAngle", swerveModuleStates[0].angle.getDegrees());
    SmartDashboard.putNumber("FrontRightAngle", swerveModuleStates[1].angle.getDegrees());
    SmartDashboard.putNumber("BackLeftAngle", swerveModuleStates[2].angle.getDegrees());
    SmartDashboard.putNumber("BackRightAngle", swerveModuleStates[3].angle.getDegrees());

    SmartDashboard.putNumber("FrontLeftSet", frontLeft.getDriveMotorEncoderVelocity());
    SmartDashboard.putNumber("FrontRightSet", frontRight.getDriveMotorEncoderVelocity());
    SmartDashboard.putNumber("BackLeftSet", backLeft.getDriveMotorEncoderVelocity());
    SmartDashboard.putNumber("BackRightSet", backRight.getDriveMotorEncoderVelocity());

    SmartDashboard.putNumber("FrontLeftCANcoderAngle", Math.toDegrees(frontLeft.getCANcoderRadians()));
    SmartDashboard.putNumber("FrontRightCANcoderAngle", Math.toDegrees(frontRight.getCANcoderRadians()));
    SmartDashboard.putNumber("BackLeftCANcoderAngle", Math.toDegrees(backLeft.getCANcoderRadians()));
    SmartDashboard.putNumber("BackRightCANcoderAngle", Math.toDegrees(backRight.getCANcoderRadians()));

    SmartDashboard.putNumber("FrontLeftMotorEncoderAngle", frontLeft.getSteerMotorEncoderAngle());
    SmartDashboard.putNumber("FrontRightMotorEncoderAngle", frontRight.getSteerMotorEncoderAngle());
    SmartDashboard.putNumber("BackLeftMotorEncoderAngle", backLeft.getSteerMotorEncoderAngle());
    SmartDashboard.putNumber("BackRightMotorEncoderAngle", backRight.getSteerMotorEncoderAngle());
  }

}
