// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  private ShuffleboardTab limelightTab;

  private GenericEntry redXPose;
  private GenericEntry redYPose;
  private GenericEntry redYaw;

  private GenericEntry blueXPose;
  private GenericEntry blueYPose;
  private GenericEntry blueYaw;
  
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    NetworkTable tableOne = NetworkTableInstance.getDefault().getTable("limelight-one");
    NetworkTable tableTwo = NetworkTableInstance.getDefault().getTable("limelight-two");

    limelightTab = Shuffleboard.getTab("Limelight Tab");
    redXPose = limelightTab.add("Red X Pose", 0).getEntry();
    redYPose = limelightTab.add("Red Y Pose", 0).getEntry();
    redYaw = limelightTab.add("Red Yaw Angle", 0).getEntry();

    blueXPose = limelightTab.add("Blue X Pose", 0).getEntry();
    blueYPose = limelightTab.add("Blue Y Pose", 0).getEntry();
    blueYaw = limelightTab.add("Blue Yaw Angle", 0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    redXPose.setDouble(getRedBotPoseX("limelight-two"));
    redYPose.setDouble(getRedBotPoseY("limelight-two"));
    redYaw.setDouble(getRedBotPoseYaw("limelight-two"));

    blueXPose.setDouble(getBlueBotPoseX("limelight-two"));
    blueYPose.setDouble(getBlueBotPoseY("limelight-two"));
    blueYaw.setDouble(getBlueBotPoseYaw("limelight-two"));
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
    return LimelightHelpers.getBotPose(limelightHostName);
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

  /**
   * Gets the robot's position in 3d space relative to the red alliance driver stations
   * 
   * @return 3d bot pose array values
   */
  public static double[] getRed3dBotPose(String limelightHostName) {
    return LimelightHelpers.getBotPose_wpiRed(limelightHostName);
  }

  public static double getRedBotPoseX(String limelightHostName) {
    double botPoseArray[] = getRed3dBotPose(limelightHostName);
    return botPoseArray[0];
  }

  public static double getRedBotPoseY(String limelightHostName) {
    double botPoseArray[] = getRed3dBotPose(limelightHostName);
    return botPoseArray[1];
  }

  public static double getRedBotPoseZ(String limelightHostName) {
    double botPoseArray[] = getRed3dBotPose(limelightHostName);
    return botPoseArray[2];
  }

  public static double getRedBotPoseRoll(String limelightHostName) {
    double botPoseArray[] = getRed3dBotPose(limelightHostName);
    return botPoseArray[3];
  }

  public static double getRedBotPosePitch(String limelightHostName) {
    double botPoseArray[] = getRed3dBotPose(limelightHostName);
    return botPoseArray[4];
  }

  public static double getRedBotPoseYaw(String limelightHostName) {
    double botPoseArray[] = getRed3dBotPose(limelightHostName);
    return botPoseArray[5];
  }

  /**
   * Gets the robot's position in 3d space relative to the blue alliance driver stations
   * 
   * @return 3d bot pose array values
   */
  public static double[] getBlue3dBotPose(String limelightHostName) {
    return LimelightHelpers.getBotPose_wpiBlue(limelightHostName);
  }

  public static double getBlueBotPoseX(String limelightHostName) {
    double botPoseArray[] = getBlue3dBotPose(limelightHostName);
    return botPoseArray[0];
  }

  public static double getBlueBotPoseY(String limelightHostName) {
    double botPoseArray[] = getBlue3dBotPose(limelightHostName);
    return botPoseArray[1];
  }

  public static double getBlueBotPoseZ(String limelightHostName) {
    double botPoseArray[] = getBlue3dBotPose(limelightHostName);
    return botPoseArray[2];
  }

  public static double getBlueBotPoseRoll(String limelightHostName) {
    double botPoseArray[] = getBlue3dBotPose(limelightHostName);
    return botPoseArray[3];
  }

  public static double getBlueBotPosePitch(String limelightHostName) {
    double botPoseArray[] = getBlue3dBotPose(limelightHostName);
    return botPoseArray[4];
  }

  public static double getBlueBotPoseYaw(String limelightHostName) {
    double botPoseArray[] = getBlue3dBotPose(limelightHostName);
    return botPoseArray[5];
  }


  // APRILTAG SET METHODS \\

  public static void setAprilTagPipeline(String limelightHostName) {
    LimelightHelpers.setPipelineIndex(limelightHostName, 0);
  }


  // RETROREFLECTIVE/CUBE/CONE GET METHODS \\

  /**
   * Returns true if a retroreflective/cube/cone target is detected
   * 
   * @param limelightHostName Host name for the correct Limelight
   * @return true if a retroreflective target is detected, false if not
   */
  public static boolean isTargetDetected(String limelightHostName) {
    if (NetworkTableInstance.getDefault().getTable(limelightHostName).getEntry("tv").getDouble(0) == 1) {
      return true;
    } else {
      return false;
    }
  }

  /** Works for retroreflective, cube, and cone (depending on which pipeline is set) */
  public static double getX(String limelightHostName) {
    return LimelightHelpers.getTX(limelightHostName);
  }

  /** Works for retroreflective, cube, and cone (depending on which pipeline is set) */
  public static double getY(String limelightHostName) {
    return LimelightHelpers.getTY(limelightHostName);
  }


  // RETROREFLECTIVE SET METHODS \\

  public static void setRetroreflectivePipeline(String limelightHostName) {
    LimelightHelpers.setPipelineIndex(limelightHostName, 1);
  }

  // CUBE/CONE GET METHODS \\

  public static void setConePipeline(String limelightHostName) {
    LimelightHelpers.setPipelineIndex(limelightHostName, 2);
  }

  public static void setCubePipeline(String limelightHostName) {
    LimelightHelpers.setPipelineIndex(limelightHostName, 3);
  }

  // CUBE/CONE SET METHODS \\

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
