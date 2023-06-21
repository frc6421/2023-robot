// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriverControlSystem;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  public final SwerveDriveKinematics swerveKinematics;

  //private final SwerveDriveOdometry odometry;

  private final SwerveDrivePoseEstimator swervePoseEstimator;
  private final Matrix<N3, N1> stateStandardDeviations;
  private final Matrix<N3, N1> visionStandardDeviations;

  private final PIDController angleController;
  private final PIDController driftCorrector;

  // Creates a sendable chooser on smartdashboard to select the desired control
  // system
  private SendableChooser<DriverControlSystem> controlSystem;

  // Creates the slew rates to slowly accelerate controller inputs
  private SlewRateLimiter magnitudeSlewRate;
  private SlewRateLimiter xDriveSlew;
  private SlewRateLimiter yDriveSlew;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontLeft = new SwerveModule(ModuleConstants.FRONT_LEFT_MODULE_DRIVE_CAN_ID,
        ModuleConstants.FRONT_LEFT_MODULE_STEER_CAN_ID, ModuleConstants.FRONT_LEFT_MODULE_ENCODER_CAN_ID,
        ModuleConstants.FRONT_LEFT_MODULE_ANGLE_OFFSET);
    frontRight = new SwerveModule(ModuleConstants.FRONT_RIGHT_MODULE_DRIVE_CAN_ID,
        ModuleConstants.FRONT_RIGHT_MODULE_STEER_CAN_ID, ModuleConstants.FRONT_RIGHT_MODULE_ENCODER_CAN_ID,
        ModuleConstants.FRONT_RIGHT_MODULE_ANGLE_OFFSET);
    backLeft = new SwerveModule(ModuleConstants.BACK_LEFT_MODULE_DRIVE_CAN_ID,
        ModuleConstants.BACK_LEFT_MODULE_STEER_CAN_ID, ModuleConstants.BACK_LEFT_MODULE_ENCODER_CAN_ID,
        ModuleConstants.BACK_LEFT_MODULE_ANGLE_OFFSET);
    backRight = new SwerveModule(ModuleConstants.BACK_RIGHT_MODULE_DRIVE_CAN_ID,
        ModuleConstants.BACK_RIGHT_MODULE_STEER_CAN_ID, ModuleConstants.BACK_RIGHT_MODULE_ENCODER_CAN_ID,
        ModuleConstants.BACK_RIGHT_MODULE_ANGLE_OFFSET);

    swerveKinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
            DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Front right
        new Translation2d(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
            -DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Back left
        new Translation2d(-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
            DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Back right
        new Translation2d(-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
            -DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0));

    // odometry = new SwerveDriveOdometry(swerveKinematics, GyroSubsystem.getYawAngle(), new SwerveModulePosition[] {
    //     frontLeft.getModulePosition(),
    //     frontRight.getModulePosition(),
    //     backLeft.getModulePosition(),
    //     backRight.getModulePosition() });

    // State Standard Deviations \\
    stateStandardDeviations = new Matrix<>(Nat.N3(), Nat.N1()); //TODO tune standard deviation values
    // Set X standard deviation
    stateStandardDeviations.set(0, 0, 0.05);
    // Set Y standard deviation
    stateStandardDeviations.set(1, 0, 0.05);
    // Set yaw standard deviation
    stateStandardDeviations.set(2, 0, 5);

    // Vision Standard Deviations \\
    visionStandardDeviations = new Matrix<>(Nat.N3(), Nat.N1());  //TODO tune standard deviation values
    // Set X standard deviation
    visionStandardDeviations.set(0, 0, 0.5);
    // Set Y standard deviation
    visionStandardDeviations.set(1, 0, 0.5);
    // Set yaw standard deviation
    visionStandardDeviations.set(2, 0, 30);

    swervePoseEstimator = new SwerveDrivePoseEstimator(swerveKinematics,
      GyroSubsystem.getYawAngle(), 
      new SwerveModulePosition[] {
        frontLeft.getModulePosition(),
        frontRight.getModulePosition(),
        backLeft.getModulePosition(),
        backRight.getModulePosition()
      }, 
      new Pose2d(), //TODO update starting pose?
      stateStandardDeviations,
      visionStandardDeviations);

    // PID controller for the rotation of the robot
    angleController = new PIDController(DriveConstants.ANGLE_CONTROLLER_KP, 0, 0);
    angleController.enableContinuousInput(-180, 180);

    driftCorrector = new PIDController(.001, 0, 0); // TODO implement Feed Forward for functionality
    driftCorrector.enableContinuousInput(0, 360);

    // Sets up the sendable chooser on SmartDashboard to select control system
    controlSystem = new SendableChooser<>();
    controlSystem.setDefaultOption("Left Trigger Controls", DriverControlSystem.LEFT_TRIGGER);
    controlSystem.addOption("Joystick Controls", DriverControlSystem.JOYSTICK); // TODO Get these into an enum w/ switch
    controlSystem.addOption("RightTrigger", DriverControlSystem.RIGHT_TRIGGER);
    SmartDashboard.putData("Control system", controlSystem);

    magnitudeSlewRate = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
    xDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
    yDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Field coordinates are different in auto and teleop because autonomous commands ignore the loading zones
    if(DriverStation.isAutonomous()) {
      swervePoseEstimator.update(GyroSubsystem.getYawAngle(), new SwerveModulePosition[] {
          frontLeft.getModulePosition(),
          frontRight.getModulePosition(),
          backLeft.getModulePosition(),
          backRight.getModulePosition()
        });
    } else if(DriverStation.isTeleop()) {
      
    }

    // odometry.update(GyroSubsystem.getYawAngle(), new SwerveModulePosition[] {
    //   frontLeft.getModulePosition(),
    //   frontRight.getModulePosition(),
    //   backLeft.getModulePosition(),
    //   backRight.getModulePosition()
    //   }
    //);
  }

  // ODOMETRY METHODS \\

  /**
   * Returns estimated current robot pose in meters
   * 
   * @return current robot pose2d in meters
   */
  public Pose2d getPose2d() {
    //return odometry.getPoseMeters();
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Returns estimated current robot heading as a Rotation2d
   * 
   * @return current estimated robot heading as a Rotation2d (radians)
   */
  public Rotation2d getPoseHeading() {
    //return odometry.getPoseMeters().getRotation();
    return swervePoseEstimator.getEstimatedPosition().getRotation();
  }

  //TODO rename method if using pose estimator
  /**
   * Resets odometry to the given pose value
   * 
   * @param pose to use for reset
   */
  public void resetOdometry(Pose2d pose) {
    // odometry.resetPosition(GyroSubsystem.getYawAngle(), new SwerveModulePosition[] {
    //     frontLeft.getModulePosition(),
    //     frontRight.getModulePosition(),
    //     backLeft.getModulePosition(),
    //     backRight.getModulePosition() }, pose);
    swervePoseEstimator.resetPosition(GyroSubsystem.getYawAngle(), new SwerveModulePosition[] {
        frontLeft.getModulePosition(),
        frontRight.getModulePosition(),
        backLeft.getModulePosition(),
        backRight.getModulePosition()
      }, pose);
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
   * Sets the swerve module states
   * Uses closed loop control for auto
   * 
   * @param desiredStates the desired swerve module states
   */
  public void autoSetModuleStates(SwerveModuleState[] desiredStates) {
    // Ensures all wheels obey max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    // Sets the swerve modules to their desired states using optimization method
    frontLeft.autoSetDesiredState(desiredStates[0]);
    frontRight.autoSetDesiredState(desiredStates[1]);
    backLeft.autoSetDesiredState(desiredStates[2]);
    backRight.autoSetDesiredState(desiredStates[3]);
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
  public void setSteerMotorsToAbsolute() {
    frontRight.setSteerMotorToAbsolute();
    frontLeft.setSteerMotorToAbsolute();
    backLeft.setSteerMotorToAbsolute();
    backRight.setSteerMotorToAbsolute();
  }

  /**
   * Sets up the drive method
   * 
   * @param xSpeedInput    value from -1.0 to 1.0 to convert to x-direction meters
   *                       per second
   * @param ySpeedInput    value from -1.0 to 1.0 to convert to y-direction meters
   *                       per second
   * @param rotationInput  value from -1.0 to 1.0 to convert to rotational speed
   *                       in radians per second
   * @param leftMagnitude  value from 0 to 1 returned by the left trigger to set
   *                       the magnitude of
   *                       x and y speeds for left trigger controls(not
   *                       rotational)
   * @param rightMagnitude value from 0 to 1 returned by the right trigger to set
   *                       the magnitude of
   *                       x and y speeds for right trigger controls(not
   *                       rotational)
   */
  public void drive(double xSpeedInput, double ySpeedInput, double rotationInput, double leftMagnitude,
      double rightMagnitude) {

    ySpeedInput = MathUtil.applyDeadband(ySpeedInput, ModuleConstants.PERCENT_DEADBAND);
    xSpeedInput = MathUtil.applyDeadband(xSpeedInput, ModuleConstants.PERCENT_DEADBAND);
    rotationInput = MathUtil.applyDeadband(rotationInput, ModuleConstants.PERCENT_DEADBAND);
    leftMagnitude = MathUtil.applyDeadband(leftMagnitude, ModuleConstants.PERCENT_DEADBAND);
    rightMagnitude = MathUtil.applyDeadband(rightMagnitude, ModuleConstants.PERCENT_DEADBAND);

    Rotation2d speeds = new Rotation2d(ySpeedInput, xSpeedInput);
    double xSpeed;
    double ySpeed;

    switch (controlSystem.getSelected()) {

      case RIGHT_TRIGGER:
        rightMagnitude = magnitudeSlewRate.calculate(rightMagnitude);
        xSpeed = speeds.getSin() * rightMagnitude;
        ySpeed = speeds.getCos() * rightMagnitude;
        break;

      case JOYSTICK:
        xSpeed = xDriveSlew.calculate(xSpeedInput);
        ySpeed = yDriveSlew.calculate(ySpeedInput);
        break;

      case LEFT_TRIGGER:
      default:
        leftMagnitude = magnitudeSlewRate.calculate(leftMagnitude);
        xSpeed = speeds.getSin() * leftMagnitude;
        ySpeed = speeds.getCos() * leftMagnitude;
        break;
    }

    //Keeps the robot from moving with no joystick inputs
    if(Math.abs(ySpeedInput) < ModuleConstants.PERCENT_DEADBAND){
      ySpeed = 0;
    }
    if(Math.abs(xSpeedInput) < ModuleConstants.PERCENT_DEADBAND){
      xSpeed = 0;
    }
    if(Math.abs(rotationInput) < ModuleConstants.PERCENT_DEADBAND){
      rotationInput = 0;
    }

    xSpeed = -1 * (xSpeed) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    ySpeed = -1 * (ySpeed) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    //double rotation = -1 * (rotationInput + Math.signum(rotationInput) * .095) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // Input squaring
    //xSpeed = -1 * Math.signum(xSpeed) * (xSpeed * xSpeed) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    //ySpeed = -1 * Math.signum(ySpeed) * (ySpeed * ySpeed) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    double rotation = -1 * Math.signum(rotationInput) * (rotationInput * rotationInput) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // Sets chassis speeds
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation,
        GyroSubsystem.getYawAngle());
  
    // Sets field relative speeds to the swerve module states
    var swerveModuleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Ensures all wheels obey max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);

    // Turns all the wheels inward to prevent pushing and sets the speed of each
    // module to 0
    // if (driverController.povDown().getAsBoolean()) {
    //   swerveModuleStates[0].angle = Rotation2d.fromDegrees(45);
    //   swerveModuleStates[0].speedMetersPerSecond = 0;
    //   swerveModuleStates[1].angle = Rotation2d.fromDegrees(-45);
    //   swerveModuleStates[1].speedMetersPerSecond = 0;
    //   swerveModuleStates[2].angle = Rotation2d.fromDegrees(-45);
    //   swerveModuleStates[2].speedMetersPerSecond = 0;
    //   swerveModuleStates[3].angle = Rotation2d.fromDegrees(45);
    //   swerveModuleStates[3].speedMetersPerSecond = 0;
    // }

    // Sets the swerve modules to their desired states using optimization method
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Same as drive method, but without the trigger control inputs.
   *  This allows it to be used for driving to targets based on vision.
   * 
   * @param xSpeedInput percent input from -1 to 1 (converts to meters per sec)
   * @param ySpeedInput percent input from -1 to 1 (converts to meters per sec)
   * @param rotationInput percent input from -1 to 1 (converts to radians per sec)
   */
  public void autoDrive(double xSpeedInput, double ySpeedInput, double rotationInput) {
    // double xSpeed = xSpeedInput * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    // double ySpeed = ySpeedInput * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;

    // double rotation = rotationInput * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    double xSpeed = xSpeedInput * (DriveConstants.MAX_VELOCITY_METERS_PER_SECOND / 2) + Math.signum(xSpeedInput) * 0.18;
    double ySpeed = ySpeedInput * (DriveConstants.MAX_VELOCITY_METERS_PER_SECOND / 2) + Math.signum(ySpeedInput) * 0.18;

    double rotation = rotationInput * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND + Math.signum(rotationInput) * 0.18;
    
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
