// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutoConstants {

  }

  public static class DriveConstants {
    /**
     * Drivetrain trackwidth from side to side
     * 
     * Measured from the center of the wheels on each side
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(18.75);

    /**
     * Drivetrain wheelbase from front to back
     * 
     * Measured from the center of the wheels on each side
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.75); 

    public static final double GEAR_RATIO_MOTOR_TO_WHEEL = 6.75;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final int COUNTS_PER_ROTATION = 2048;

    public static final double STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE = (STEER_GEAR_RATIO * COUNTS_PER_ROTATION) / 360;
    
    public static final double DISTANCE_PER_ENCODER_COUNT = WHEEL_CIRCUMFERENCE / (COUNTS_PER_ROTATION * GEAR_RATIO_MOTOR_TO_WHEEL);

    // Formula for calculating theoretical max velocity:
    // Motor free speed RPM / 60 * Drive reduction * Wheel diameter meters * pi
    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // Constant for SDS MK4i Modules
    public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0) * DRIVE_REDUCTION * DriveConstants.WHEEL_CIRCUMFERENCE;
    public static final double DRIVE_VOLTAGE = 9.5; //TODO update with correct voltage
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2);
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI; //TODO test value instead of theoretical value
    public static final double DRIVE_NERF_JOYSTICK_MULTIPLIER = 0.75;

    public static final double S_VOLTS = 0.60043;
    public static final double V_VOLT_SECONDS_PER_METER = 2.2591;
    public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.17289;

    public static final double DRIVE_SLEW_RATE = 2;

    public static final double ANGLE_CONTROLLER_KP = .0014;
  }

  public static class ModuleConstants {
    //TODO update for CANivore
    public static final String CANIVORE_NAME = "driveBus";

    public static final String RIO_NAME = "rio";

    public static final double PERCENT_DEADBAND = .075;
    
    //TODO update angle offsets on competition robot
    public static final int FRONT_LEFT_MODULE_DRIVE_CAN_ID = 12;
    public static final int FRONT_LEFT_MODULE_STEER_CAN_ID = 13;
    public static final int FRONT_LEFT_MODULE_ENCODER_CAN_ID = 13;
    public static final double FRONT_LEFT_MODULE_ANGLE_OFFSET = -171.6;

    public static final int FRONT_RIGHT_MODULE_DRIVE_CAN_ID = 10;
    public static final int FRONT_RIGHT_MODULE_STEER_CAN_ID = 11;
    public static final int FRONT_RIGHT_MODULE_ENCODER_CAN_ID = 11;
    public static final double FRONT_RIGHT_MODULE_ANGLE_OFFSET = -137.0;

    public static final int BACK_LEFT_MODULE_DRIVE_CAN_ID = 16;
    public static final int BACK_LEFT_MODULE_STEER_CAN_ID = 17;
    public static final int BACK_LEFT_MODULE_ENCODER_CAN_ID = 17;
    public static final double BACK_LEFT_MODULE_ANGLE_OFFSET = -192.5;

    public static final int BACK_RIGHT_MODULE_DRIVE_CAN_ID = 14;
    public static final int BACK_RIGHT_MODULE_STEER_CAN_ID = 15;
    public static final int BACK_RIGHT_MODULE_ENCODER_CAN_ID = 15;
    public static final double BACK_RIGHT_MODULE_ANGLE_OFFSET = -352.5;

    public static final double MODULE_DRIVE_P = 0.208; 
    public static final double MODULE_DRIVE_I = 0; 
    public static final double MODULE_DRIVE_D = 0; 

    public static final double MODULE_STEER_P = 0.3; 
    public static final double MODULE_STEER_I = 0; 
    public static final double MODULE_STEER_D = 0; 

    public static final double DEADBAND_DRIVE_MOTOR = 0.02;
    public static final double DEADBAND_STEER_MOTOR = 0.02;
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static enum driverControlSystem {
    LEFT_TRIGGER,
    RIGHT_TRIGGER,
    JOYSTICK,
  }

}
