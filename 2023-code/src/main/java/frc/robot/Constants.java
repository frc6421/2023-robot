// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutoConstants {
    public static final double AUTO_MAX_VELOCITY_METERS_PER_SECOND = 1; // TODO update 4
    public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1; // TODO update 8

    public static final double AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 2 * Math.PI;
    public static final double AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC = Math.PI;

    public static final double X_DRIVE_P = 2.1821;
    public static final double X_DRIVE_I = 0;
    public static final double X_DRIVE_D = 0;

    public static final double Y_DRIVE_P = 2.1821;
    public static final double Y_DRIVE_I = 0;
    public static final double Y_DRIVE_D = 0;

    public static final double THETA_P = 1.2;
    public static final double THETA_I = 0;
    public static final double THETA_D = 0;

    public static class TrajectoryConstants {
      public static final double CENTER_OF_ROBOT_LENGTH = Units.inchesToMeters(16.25);
      public static final double CENTER_OF_ROBOT_WIDTH = Units.inchesToMeters(15.25);

      // Note: AprilTag coordinate system starts at right corner of blue alliance
      // driver stations
      // (0, 0) is the front right corner of the scoring grids
      public static final Translation2d ORIGIN = new Translation2d(0, 0);

      // First game piece is farthest to the left when standing in driver station,
      // fourth is farthest to the right
      public static final Translation2d FOURTH_GAME_PIECE = new Translation2d(Units.feetToMeters(18.67),
          Units.feetToMeters(3));
      public static final Translation2d THIRD_GAME_PIECE = new Translation2d(Units.feetToMeters(18.67),
          Units.feetToMeters(7));
      public static final Translation2d SECOND_GAME_PIECE = new Translation2d(Units.feetToMeters(18.67),
          Units.feetToMeters(11));
      public static final Translation2d FIRST_GAME_PIECE = new Translation2d(Units.feetToMeters(18.67),
          Units.feetToMeters(15));

      /** In line with fourth game piece */
      public static final Translation2d FAR_EDGE_OF_COMMUNITY = new Translation2d(Units.inchesToMeters(132.81),
          Units.feetToMeters(3));

      // First cone node is farthest to the left on right grid when standing in driver
      // station
      public static final Translation2d SECOND_CONE_NODE = new Translation2d(Units.feetToMeters(0),
          Units.inchesToMeters(20));
      public static final Translation2d FIRST_CONE_NODE = new Translation2d(Units.feetToMeters(0),
          Units.inchesToMeters(64));

      // Cone node on the right side of the coopertition grid
      public static final Translation2d SECOND_COOPERTITION_CONE_NODE = new Translation2d(Units.feetToMeters(0),
          Units.inchesToMeters(86));

      public static final Translation2d CUBE_NODE = new Translation2d(Units.feetToMeters(0),
          Units.inchesToMeters(42));

      public static final Translation2d MID_POINT_OF_PIECES_AND_CHARGE_STATION = new Translation2d(
          Units.feetToMeters(139.32),
          Units.inchesToMeters(107.39));
      public static final Translation2d CENTER_OF_CHARGE_STATION = new Translation2d(Units.inchesToMeters(96.75),
          Units.inchesToMeters(107.39));

      // Flipped Y value trajectories (for left start)

      // First game piece is farthest to the left when standing in driver station,
      // fourth is farthest to the right
      public static final Translation2d FLIPPED_FOURTH_GAME_PIECE = new Translation2d(Units.feetToMeters(18.67),
          -Units.feetToMeters(3));
      public static final Translation2d FLIPPED_THIRD_GAME_PIECE = new Translation2d(Units.feetToMeters(18.67),
          -Units.feetToMeters(7));
      public static final Translation2d FLIPPED_SECOND_GAME_PIECE = new Translation2d(Units.feetToMeters(18.67),
          -Units.feetToMeters(11));
      public static final Translation2d FLIPPED_FIRST_GAME_PIECE = new Translation2d(Units.feetToMeters(18.67),
          -Units.feetToMeters(15));

      /** In line with fourth game piece */
      public static final Translation2d FLIPPED_FAR_EDGE_OF_COMMUNITY = new Translation2d(
          Units.inchesToMeters(132.81),
          -Units.feetToMeters(3));

      // First is farthest to the right on first grid (flipped relative to normal
      // trajectories)
      public static final Translation2d FLIPPED_SECOND_CONE_NODE = new Translation2d(Units.feetToMeters(0),
          -Units.inchesToMeters(20));
      public static final Translation2d FLIPPED_FIRST_CONE_NODE = new Translation2d(Units.feetToMeters(0),
          -Units.inchesToMeters(64));

      public static final Translation2d FLIPPED_SECOND_COOPERTITION_CONE_NODE = new Translation2d(Units.feetToMeters(0),
          -Units.inchesToMeters(86));

      public static final Translation2d FLIPPED_CUBE_NODE = new Translation2d(Units.feetToMeters(0),
          -Units.inchesToMeters(42));

      public static final Translation2d FLIPPED_CENTER_OF_CHARGE_STATION = new Translation2d(
          Units.inchesToMeters(96.75),
          -Units.inchesToMeters(107.39));

      public static final Translation2d FLIPPED_MID_POINT_OF_PIECES_AND_CHARGE_STATION = new Translation2d(
          Units.feetToMeters(139.32),
          -Units.inchesToMeters(107.39));
    }
  }

  public static class BlinkinConstants {
    public static final double BLINKIN_RED = 0.61;
    public static final double BLINKIN_BLUE = 0.87;
    public static final double BLINKIN_YELLOW = 0.69;
    public static final double BLINKIN_VIOLET = 0.91;
    public static final double BLINKIN_RAINBOW = -0.99;
    public static final double BLINKIN_HOT_PINK = 0.57;
    public static final double BLINKIN_GREEN = 0.77;
    public static final double BLINKIN_FADE_TO_BLACK = -0.03;
    public static final double BLINKIN_RAINBOW_WAVE = -0.45;
    public static final double BLINKIN_RAINBOW_SINELON = -0.45;
    public static final double BLINKIN_CONFETTI = -0.87;
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

    public static final double STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE = (STEER_GEAR_RATIO * COUNTS_PER_ROTATION)
        / 360;

    public static final double DISTANCE_PER_ENCODER_COUNT = WHEEL_CIRCUMFERENCE
        / (COUNTS_PER_ROTATION * GEAR_RATIO_MOTOR_TO_WHEEL);

    // Formula for calculating theoretical max velocity:
    // Motor free speed RPM / 60 * Drive reduction * Wheel diameter meters * pi
    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // Constant for SDS MK4i Modules
    public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0) * DRIVE_REDUCTION * DriveConstants.WHEEL_CIRCUMFERENCE;
    public static final double DRIVE_VOLTAGE = 9.5; //TODO update with correct voltage
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2);
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI; //TODO test value instead of theoretical value
    public static final double DRIVE_NERF_JOYSTICK_MULTIPLIER = 0.75;

    public static final double S_VOLTS = 0.01690;
    public static final double V_VOLT_SECONDS_PER_METER = 0.1791;
    public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.05486;

    public static final double DRIVE_SLEW_RATE = 2.5;

    public static final double ANGLE_CONTROLLER_KP = 0.0014;

    public static final int GYRO_CAN_ID = 30;

    public static final double GYRO_PITCH_OFFSET = -0.056;
    public static final double GYRO_ROLL_OFFSET = 0.102;
    public static final double GYRO_YAW_OFFSET = -0.435;
  }

  public static class ModuleConstants {
    public static final String CANIVORE_NAME = "driveBus";

    public static final String RIO_NAME = "rio";

    public static final double PERCENT_DEADBAND = 0.045;
    
    //TODO update angle offsets on competition robot
    public static final int FRONT_LEFT_MODULE_DRIVE_CAN_ID = 12;
    public static final int FRONT_LEFT_MODULE_STEER_CAN_ID = 13;
    public static final int FRONT_LEFT_MODULE_ENCODER_CAN_ID = 13;
    public static final double FRONT_LEFT_MODULE_ANGLE_OFFSET = -62.05; 

    public static final int FRONT_RIGHT_MODULE_DRIVE_CAN_ID = 10;
    public static final int FRONT_RIGHT_MODULE_STEER_CAN_ID = 11;
    public static final int FRONT_RIGHT_MODULE_ENCODER_CAN_ID = 11;
    public static final double FRONT_RIGHT_MODULE_ANGLE_OFFSET = -350.51; 

    public static final int BACK_LEFT_MODULE_DRIVE_CAN_ID = 16;
    public static final int BACK_LEFT_MODULE_STEER_CAN_ID = 17;
    public static final int BACK_LEFT_MODULE_ENCODER_CAN_ID = 17;
    public static final double BACK_LEFT_MODULE_ANGLE_OFFSET = -108.2;

    public static final int BACK_RIGHT_MODULE_DRIVE_CAN_ID = 14;
    public static final int BACK_RIGHT_MODULE_STEER_CAN_ID = 15;
    public static final int BACK_RIGHT_MODULE_ENCODER_CAN_ID = 15;
    public static final double BACK_RIGHT_MODULE_ANGLE_OFFSET = -302.43;

    public static final double MODULE_DRIVE_P = 0.208;
    public static final double MODULE_DRIVE_I = 0;
    public static final double MODULE_DRIVE_D = 0;

    public static final double MODULE_STEER_P = 0.3;
    public static final double MODULE_STEER_I = 0;
    public static final double MODULE_STEER_D = 0;

    public static final double DEADBAND_DRIVE_MOTOR = 0.02;
    public static final double DEADBAND_STEER_MOTOR = 0.02;
  }

  public static class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_CAN_ID = 50;

    public static final double ELEVATOR_P = 60; // 1/21/23 tuned with no arm weight
    public static final double ELEVATOR_I = 0; 
    public static final double ELEVATOR_D = 0;

    public static final double ELEVATOR_FF = 0.02; // 2/7/23 tuned with no arm weight

    public static final double ELEVATOR_GEAR_RATIO = 15;

    public static final double ELEVATOR_MAX_POS_IN = 20.375;
    public static final double ELEVATOR_MIN_POS_IN = 0;

    public static final float ELEVATOR_FORWARD_SOFT_LIMIT_METERS = (float)Units.inchesToMeters(ELEVATOR_MAX_POS_IN);
    public static final float ELEVATOR_REVERSE_SOFT_LIMIT = 0f;

    public static final double ELEVATOR_DEFULT_NERF = 0.01;

    public static final double ELEVATOR_SUBSTATION_LENGTH = 0.27;

    /**
     * In meters
     */
    public static final double ELEVATOR_SPROCKET_PITCH_CIRCUMFERENCE = (Units.inchesToMeters(1.7567) * Math.PI);
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int COPILOT_CONTROLLER_PORT = 1;
  }

  public static class ArmConstants {
    public static final int ARM_CAN_ID = 40;

    public static final double ARM_DEFAULT_FF = 0.04; // Determined on 2/13/23 0.04    
    public static final double ARM_P = 0.06;// Determined on 2/7/23 0.06
    public static final double ARM_I = 0.0;
    public static final double ARM_D = 0.0;

    public static final float ARM_SOFT_LIMIT = 0.0f;

    public static final double ARM_GEAR_RATIO = 120.0;

    /**
     * Calculated degrees arm moves per motor rotation from 360 / gear ratio
     */
    public static final double DEGREES_PER_MOTOR_ROTATION = (360 / ARM_GEAR_RATIO);

    public static final float ARM_IN_SOFT_LIMIT = -29;

    public static final float ARM_OUT_SOFT_LIMIT = 224;

    public static final float ARM_ELEVATOR_OUT_SOFT_LIMIT = 270;

    public static final double ARM_MAX_TEST_PERCENT_OUTPUT = 0.15;

    // public static final int ARM_POS_HORIZONTAL = 840; // TODO update with correct
    // value(May use later)

    public static final double MAX_ARM_GRAVITY_FF = 0.0375; // Determined on 2/7/2023

    public static final double FLOOR_MIN_INCH_DISTANCE = 12.1;

    public static final double FLOOR_MAX_INCH_DISTANCE = 20.6;

    public static final boolean ARM_IS_INVERTED = true;

    public static final double ARM_SET_POS_CONVERSION_FACTOR = 1.6;

    public static class ArmAngleConstants {
      public static final double CONE_HIGH_MIN_ANGLE = 155.0;
      public static final double CONE_HIGH_MAX_ANGLE = 142.3;
      public static final double CONE_HIGH_TOP_ANGLE = 130.0;
      
      public static final double CONE_MID_MIN_ANGLE = 153.5;
      public static final double CONE_MID_MAX_ANGLE = 141.8;
      public static final double CONE_MID_TOP_ANGLE = 135.0;

      public static final double CUBE_HIGH_MIN_ANGLE = 181.9;
      public static final double CUBE_HIGH_MAX_ANGLE = 155.3;
      public static final double CUBE_HIGH_OPTIMAL_ANGLE = 140.0;

      public static final double CUBE_MID_MIN_ANGLE = 182.7;
      public static final double CUBE_MID_MAX_ANGLE = 155.3;
      public static final double CUBE_MID_OPTIMAL_ANGLE = 140.0;

      public static final double ARM_START_POSITION = -31.0; //Determined 02/18/23 -31.0

      public static final double GRAB_FROM_SUBSTATION_ANGLE = 143.5;
      
      
      //TODO: Not final angle
      public static final double GRAB_FROM_INTAKE_ANGLE = -26.9;

      public static final double FLOOR_ANGLE = 210.7;
    }

  }

  public static enum driverControlSystem {
    LEFT_TRIGGER,
    RIGHT_TRIGGER,
    JOYSTICK,
  }

  public static class IntakeConstants {
    public static final int LEFT_INTAKE_MOTOR_ID = 20;
    public static final int RIGHT_INTAKE_MOTOR_ID = 21; 

    //TODO confer with other programmers for CAN and pneumatics
    public static final int INTAKE_PISTON_FORWARD_CHANNEL = 0;
    public static final int INTAKE_PISTON_REVERSE_CHANNEL = 1;
  }

  public static class GrabberConstants {
    public static final int FORWARD_CHANNEL = 2;
    public static final int REVERSE_CHANNEL = 3;
  }

  public class VisionConstants {
    // All distances are in meters \\
    // Left and right are relative to the driver's perspective in the driver station \\

    /** Y offset from center of the cube AprilTag */
    public static final double LEFT_CONE_OFFSET = -0.5715;
    /** Y offset from center of the cube AprilTag */
    public static final double RIGHT_CONE_OFFSET = 0.5715;
    /** X offset from substation AprilTag */
    public static final double SUBSTATION_OFFSET = 1.08;

    public static final double SUBSTATION_GAME_PIECE_Y_ANGLE = 0;

    public static final double RED_LEFT_GRID_CUBE_POSE_X = 0;
    public static final double RED_LEFT_GRID_CUBE_POSE_Y = 0;

    public static final double RED_CENTER_GRID_CUBE_POSE_X = 0;
    public static final double RED_CENTER_GRID_CUBE_POSE_Y = 0;

    public static final double RED_RIGHT_GRID_CUBE_POSE_X = 0;
    public static final double RED_RIGHT_GRID_CUBE_POSE_Y = 0;

    public static final double RED_SUBSTATION_POSE_X = 0;
    public static final double RED_SUBSTATION_POSE_Y = 0;

    public static final double BLUE_LEFT_GRID_CUBE_POSE_X = 0;
    public static final double BLUE_LEFT_GRID_CUBE_POSE_Y = 0;

    public static final double BLUE_CENTER_GRID_CUBE_POSE_X = 0;
    public static final double BLUE_CENTER_GRID_CUBE_POSE_Y = 0;

    public static final double BLUE_RIGHT_GRID_CUBE_POSE_X = 0;
    public static final double BLUE_RIGHT_GRID_CUBE_POSE_Y = 0;

    public static final double BLUE_SUBSTATION_POSE_X = 16.18;
    public static final double BLUE_SUBSTATION_POSE_Y = 6.75;
  }
}
