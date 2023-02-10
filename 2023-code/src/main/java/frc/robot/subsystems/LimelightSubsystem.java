// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  // GENERAL GET METHODS \\

  public static double getPipelineID(String limelightHostName) {
    return LimelightHelpers.getCurrentPipelineIndex(limelightHostName);
  }


  // APRILTAG GET METHODS \\

  public static double getAprilTagID(String limelightHostName) {
    return LimelightHelpers.getFiducialID(limelightHostName);
  }

  /**
   * Returns an array of the [x (meters), y (meters), z (meters), roll (degrees), pitch (degrees), yaw (degrees)]
   * 
   * @return 3D bot pose array values in field space
   */
  public static double[] get3dBotPose(String limelightHostName) {
    return LimelightHelpers.getBotpose(limelightHostName);
  }

  public static double getBotPoseX(String limelightHostName) {
    double botPoseArray[] = get3dBotPose(limelightHostName);
    return botPoseArray[0];
  }

  public static double getBotPoseY(String limelightHostName) {
    double botPoseArray[] = get3dBotPose(limelightHostName);
    return botPoseArray[1];
  }

  public static double getBotPoseZ(String limelightHostName) {
    double botPoseArray[] = get3dBotPose(limelightHostName);
    return botPoseArray[2];
  }

  public static double getBotPoseRoll(String limelightHostName) {
    double botPoseArray[] = get3dBotPose(limelightHostName);
    return botPoseArray[3];
  }

  public static double getBotPosePitch(String limelightHostName) {
    double botPoseArray[] = get3dBotPose(limelightHostName);
    return botPoseArray[4];
  }

  public static double getBotPoseYaw(String limelightHostName) {
    double botPoseArray[] = get3dBotPose(limelightHostName);
    return botPoseArray[5];
  }


  // APRILTAG SET METHODS \\

  public static void setAprilTagPipeline(String limelightHostName) {
    LimelightHelpers.setPipelineIndex(limelightHostName, 0);
  }


  // RETROREFLECTIVE GET METHODS \\

  /**
   * Returns true if a retroreflective target is detected
   * 
   * @param limelightHostName Host name for the correct Limelight
   * @return true if a retroreflective target is detected, false if not
   */
  public static boolean isRetroTargetDetected(String limelightHostName) {
    if (NetworkTableInstance.getDefault().getTable(limelightHostName).getEntry("tv").getDouble(0) == 1) {
      return true;
    } else {
      return false;
    }
  }

  public static double getX(String limelightHostName) {
    return LimelightHelpers.getTX(limelightHostName);
  }

  public static double getY(String limelightHostName) {
    return LimelightHelpers.getTY(limelightHostName);
  }


  // RETROREFLECTIVE SET METHODS \\

  public static void setRetroreflectivePipeline(String limelightHostName) {
    LimelightHelpers.setPipelineIndex(limelightHostName, 1);
  }


  // LED SET METHODS \\

  /**
   * Sets the LEDs so the pipeline settings control the LED output
   * 
   * @param limelightHostName Host name for the correct Limelight
   */
  public static void setPipelineLEDControl(String limelightHostName) {
    LimelightHelpers.setLEDMode_PipelineControl(limelightHostName);
  }

  public static void setLEDOff(String limelightHostName) {
    LimelightHelpers.setLEDMode_ForceOff(limelightHostName);
  }


}
