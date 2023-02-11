// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  // Old navx declaration
  // private final AHRS navx;

  public final SwerveDriveKinematics swerveKinematics;

  private final SwerveDriveOdometry odometry;

  private final PIDController angleController;
  private final PIDController driftCorrector;

  private double targetAngle;
  private double pXY;

  // Creates a sendable chooser on smartdashboard to select the desired control system
  private SendableChooser<String> controlSystem;

  // Creates the slew rates to slowly accelerate controller inputs
  private SlewRateLimiter magnitudeSlewRate;
  private SlewRateLimiter xDriveSlew;
  private SlewRateLimiter yDriveSlew;

  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontLeft = new SwerveModule(ModuleConstants.FRONT_LEFT_MODULE_DRIVE_CAN_ID, ModuleConstants.FRONT_LEFT_MODULE_STEER_CAN_ID, ModuleConstants.FRONT_LEFT_MODULE_ENCODER_CAN_ID, ModuleConstants.FRONT_LEFT_MODULE_ANGLE_OFFSET);
    frontRight = new SwerveModule(ModuleConstants.FRONT_RIGHT_MODULE_DRIVE_CAN_ID, ModuleConstants.FRONT_RIGHT_MODULE_STEER_CAN_ID, ModuleConstants.FRONT_RIGHT_MODULE_ENCODER_CAN_ID, ModuleConstants.FRONT_RIGHT_MODULE_ANGLE_OFFSET);
    backLeft = new SwerveModule(ModuleConstants.BACK_LEFT_MODULE_DRIVE_CAN_ID, ModuleConstants.BACK_LEFT_MODULE_STEER_CAN_ID, ModuleConstants.BACK_LEFT_MODULE_ENCODER_CAN_ID, ModuleConstants.BACK_LEFT_MODULE_ANGLE_OFFSET);
    backRight = new SwerveModule(ModuleConstants.BACK_RIGHT_MODULE_DRIVE_CAN_ID, ModuleConstants.BACK_RIGHT_MODULE_STEER_CAN_ID, ModuleConstants.BACK_RIGHT_MODULE_ENCODER_CAN_ID, ModuleConstants.BACK_RIGHT_MODULE_ANGLE_OFFSET);
    
    
    swerveKinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0, DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
      // Front right
      new Translation2d(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0, -DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
      // Back left
      new Translation2d(-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0, DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
      // Back right
      new Translation2d(-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0, -DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
    );

    odometry = new SwerveDriveOdometry(swerveKinematics, GyroSubsystem.getYawAngle(), new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()});

      //PID controller for the rotation of the robot
      angleController = new PIDController(DriveConstants.ANGLE_CONTROLLER_KP, 0, 0);
      angleController.enableContinuousInput(-180, 180);

      driftCorrector = new PIDController(.07, 0, .004);

      targetAngle = GyroSubsystem.getYawAngle().getDegrees();

      pXY = 0;

      //Sets up the sendable chooser on SmartDashboard to select control system
      controlSystem = new SendableChooser<>();
      controlSystem.setDefaultOption("Left Trigger Controls", "leftTrigger");
      controlSystem.addOption("Joystick Controls", "joystick");
      controlSystem.addOption("RightTrigger", "rightTrigger");
      SmartDashboard.putData("Control system", controlSystem);

      magnitudeSlewRate = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
      xDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
      yDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(GyroSubsystem.getYawAngle(), new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()});

      SmartDashboard.putNumber("gyro", GyroSubsystem.getYawAngle().getDegrees());

      SmartDashboard.putNumber("FrontLeftCANcoderAngle", Math.toDegrees(frontLeft.getCANcoderRadians()));
      SmartDashboard.putNumber("FrontRightCANcoderAngle", Math.toDegrees(frontRight.getCANcoderRadians()));
      SmartDashboard.putNumber("BackLeftCANcoderAngle", Math.toDegrees(backLeft.getCANcoderRadians()));
      SmartDashboard.putNumber("BackRightCANcoderAngle", Math.toDegrees(backRight.getCANcoderRadians()));
  
      SmartDashboard.putNumber("FrontLeftMotorEncoderAngle", frontLeft.getSteerMotorEncoderAngle());
      SmartDashboard.putNumber("FrontRightMotorEncoderAngle", frontRight.getSteerMotorEncoderAngle());
      SmartDashboard.putNumber("BackLeftMotorEncoderAngle", backLeft.getSteerMotorEncoderAngle());
      SmartDashboard.putNumber("BackRightMotorEncoderAngle", backRight.getSteerMotorEncoderAngle());
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
   * Returns estimated current robot heading as a Rotation2d
   * 
   * @return current estimated robot heading as a Rotation2d (radians)
   */
  public Rotation2d getPoseHeading() {
    return odometry.getPoseMeters().getRotation();
  }

  /**
   * Sets the heading to zero for auto
   * 
   * @return Rotation2d set to 0
   */
  public Rotation2d setHeadingZero() {
    return new Rotation2d(0);
  }

  /**
   * Resets odometry to the given pose value
   * 
   * @param pose to use for reset
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(GyroSubsystem.getYawAngle(), new SwerveModulePosition[] {
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

  /**
   * Sets the desired ChassisSpeeds
   * Used for auto
   * 
   * @param chassisSpeeds desired ChassisSpeeds
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, GyroSubsystem.getYawAngle());

    // Sets field relative speeds
    var swerveModuleStates = 
      swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, GyroSubsystem.getYawAngle()));
      // Ensures all wheels obey max speed
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);

      // Sets the swerve modules to their desired states using optimization method
      frontLeft.setDesiredState(swerveModuleStates[0]);
      frontRight.setDesiredState(swerveModuleStates[1]);
      backLeft.setDesiredState(swerveModuleStates[2]);
      backRight.setDesiredState(swerveModuleStates[3]);
  }

  //TODO not currently used
  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  /**
   * Sets the steer motors to the absolute encoder positions
   */
  public void setSteerMotorsToAbsolute(){
    frontRight.setSteerMotorToAbsolute();
    frontLeft.setSteerMotorToAbsolute();
    backLeft.setSteerMotorToAbsolute();
    backRight.setSteerMotorToAbsolute();
  }

   /**
   * Sets up the drive method
   * 
   * @param xSpeedInput value from -1.0 to 1.0 to convert to x-direction meters per second
   * @param ySpeedInput value from -1.0 to 1.0 to convert to y-direction meters per second
   * @param rotationInput value from -1.0 to 1.0 to convert to rotational speed in radians per second
   * @param leftMagnitude value from 0 to 1 returned by the left trigger to set the magnitude of
   * x and y speeds for left trigger controls(not rotational)
   * @param rightMagnitude value from 0 to 1 returned by the right trigger to set the magnitude of 
   * x and y speeds for right trigger controls(not rotational)
   */
  public void drive(double xSpeedInput, double ySpeedInput, double rotationInput, double leftMagnitude, double rightMagnitude) {
    Rotation2d speeds = new Rotation2d(ySpeedInput, xSpeedInput);
    double xSpeed;
    double ySpeed;  //TODO make command instead of method?
                    //TODO turn wheels to a certain x-position
    // Set speed as a percentage of our max velocity driving by left trigger
    if(controlSystem.getSelected().equals("leftTrigger")){ //TODO enum driver controls w/ switch case
      leftMagnitude = magnitudeSlewRate.calculate(leftMagnitude);

      xSpeed = speeds.getSin() * leftMagnitude * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
      ySpeed = speeds.getCos() * leftMagnitude * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }

    // Set speed as a percentage of our max velocity drivinig by right trigger
    else if(controlSystem.getSelected().equals("rightTrigger")){
      rightMagnitude = magnitudeSlewRate.calculate(rightMagnitude);
      
      xSpeed = speeds.getSin() * rightMagnitude * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
      ySpeed = speeds.getCos() * rightMagnitude * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    } //TODO Drive by voltage changes before Sussex

    // Set speed as a percentage of our max velocity driving by joystick
    else{
      xSpeed = xDriveSlew.calculate(xSpeedInput) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
      ySpeed = yDriveSlew.calculate(ySpeedInput) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }
    double rotation = rotationInput * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    //Keeps the robot from moving with no joystick inputs
    if(Math.abs(ySpeedInput) < ModuleConstants.PERCENT_DEADBAND){
      ySpeed = 0;
    }

    //Corrects the natural rotational drift of the swerve
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, GyroSubsystem.getYawAngle());
    double xy = Math.abs(chassisSpeeds.vxMetersPerSecond) + Math.abs(chassisSpeeds.vyMetersPerSecond);
    if(Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.0 || pXY <= 0){ // || pXY <= 0
      targetAngle = GyroSubsystem.getYawAngle().getDegrees();
    }
    else if(xy > 0){
      chassisSpeeds.omegaRadiansPerSecond += driftCorrector.calculate(GyroSubsystem.getYawAngle().getDegrees(), targetAngle);
    }
    pXY = xy;
    // Sets field relative speeds
    var swerveModuleStates = 
      swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, GyroSubsystem.getYawAngle()));
      // Ensures all wheels obey max speed
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);

      // Sets the swerve modules to their desired states using optimization method
      frontLeft.setDesiredState(swerveModuleStates[0]);
      frontRight.setDesiredState(swerveModuleStates[1]);
      backLeft.setDesiredState(swerveModuleStates[2]);
      backRight.setDesiredState(swerveModuleStates[3]);
  }

}
