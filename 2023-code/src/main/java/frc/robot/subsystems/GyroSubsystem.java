// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class GyroSubsystem extends SubsystemBase {
  private static WPI_Pigeon2 pigeon;

  
  /**
   * Constructs a pigeon gyro
   */
  public GyroSubsystem() 
  {
    pigeon = new WPI_Pigeon2(DriveConstants.GYRO_CAN_ID, ModuleConstants.RIO_NAME);
    pigeon.configFactoryDefault();
    zeroGyro();
  }

  @Override
  public void periodic() 
  {}

  /**
   * Get the current Rotation2D rotation of the robot
   * 
   * @return Current robot rotation in Rotation2D
   */
  public static Rotation2d getStaticRotation()
  {
    return(pigeon.getRotation2d());
  }

  /**
   * Zeros the gyro
   */
  public static void zeroGyro()
  {
    pigeon.setYaw(DriveConstants.GYRO_YAW_OFFSET);
  }

  /**
   * Get rate of gyro rotation in degrees per second
   * 
   * @return turn rate in degrees per second
   */
  public static double getGyroRate() 
  {
    return pigeon.getRate();
  }

  /**
   * Gets the pitch angle of the gyro
   * 
   * @return pitch angle in Rotation2d
   */
  public static Rotation2d getPitchAngle()
  {
    return Rotation2d.fromDegrees(pigeon.getPitch());
  }

  /**
   * Gets the yaw angle of the gyro
   * 
   * @return yaw angle in Rotation2d (radians)
   */
  public static Rotation2d getYawAngle()
  {
    return Rotation2d.fromDegrees(pigeon.getYaw());
  }

  public static Rotation2d get0to360YawAngle()
  {
    return Rotation2d.fromDegrees(pigeon.getYaw());
  }

  public static double getPitchAngleDouble() {
    return pigeon.getPitch();
  }

  /**
   * Gets the roll angle of the gyro
   * 
   * @return roll angle in Rotation2d
   */
  public static Rotation2d getRollAngle()
  {
    return Rotation2d.fromDegrees(pigeon.getRoll());
  }
}
