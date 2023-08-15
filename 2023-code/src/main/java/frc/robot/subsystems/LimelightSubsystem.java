// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.CameraFilter;
import frc.robot.LimelightHelpers.LimelightResults;

public class LimelightSubsystem extends SubsystemBase {
  private ShuffleboardTab limelightTab;

  private GenericEntry redXPose;
  private GenericEntry redYPose;
  private GenericEntry redYaw;

  private GenericEntry blueXPose;
  private GenericEntry blueYPose;
  private GenericEntry blueYaw;

  private String bestCamera;

  private static SendableChooser<Enum> cameraFilterChooser = new SendableChooser<>();

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

    cameraFilterChooser.addOption("Z Pose", CameraFilter.CLOSEST_Z_POSE);
    cameraFilterChooser.addOption("Yaw Angle", CameraFilter.CLOSEST_YAW_ANGLE);
    cameraFilterChooser.addOption("Roll Angle", CameraFilter.CLOSEST_ROLL_ANGLE);
    cameraFilterChooser.addOption("Pitch Angle", CameraFilter.CLOSEST_PITCH_ANGLE);
    cameraFilterChooser.addOption("Average", CameraFilter.AVERAGE_POSE);
    cameraFilterChooser.addOption("Weighted Average", CameraFilter.WEIGHTED_AVERAGE_POSE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    redXPose.setDouble(getRedBotPoseX(bestCamera));
    redYPose.setDouble(getRedBotPoseY(bestCamera));
    redYaw.setDouble(getRedBotPoseYaw(bestCamera));

    blueXPose.setDouble(getBlueBotPoseX(bestCamera));
    blueYPose.setDouble(getBlueBotPoseY(bestCamera));
    blueYaw.setDouble(getBlueBotPoseYaw(bestCamera));

  }

  // GENERAL GET METHODS \\

  public static double getPipelineID(String limelightHostName) {
    return LimelightHelpers.getCurrentPipelineIndex(limelightHostName);
  }

  // Currently switches between different pose determination methods for testing
  // TODO remove switch case once we determine which method is best
  public static Pose2d getBestCameraPose() {
    Pose2d bestCameraPose = new Pose2d();

    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      if (isTargetDetected("limelight-one") && isTargetDetected("limelight-two")) {
        switch (cameraFilterChooser.getSelected().name()) {
          case "CLOSEST_Z_POSE":
            double zPoseOne = getRedBotPoseZ("limelight-one");
            double zPoseTwo = getRedBotPoseZ("limelight-two");

            if (zPoseOne < zPoseTwo && zPoseOne > 0) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-one");
            } else if (zPoseTwo < zPoseOne && zPoseTwo > 0) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-two");
            }

            break;
          case "CLOSEST_YAW_ANGLE":
            double yawAngleOne = getRedBotPoseYaw("limelight-one");
            double yawAngleTwo = getRedBotPoseYaw("limelight-two");

            double yawAngleDifferenceOne = Math.abs(yawAngleOne - GyroSubsystem.getYawAngle().getDegrees());
            double yawAngleDifferenceTwo = Math.abs(yawAngleTwo - GyroSubsystem.getYawAngle().getDegrees());

            if (yawAngleDifferenceOne < yawAngleDifferenceTwo) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-one");
            } else if (yawAngleDifferenceTwo < yawAngleDifferenceOne) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-two");
            }

            break;
          case "CLOSEST_ROLL_ANGLE":
            double rollAngleOne = getRedBotPoseRoll("limelight-one");
            double rollAngleTwo = getRedBotPoseRoll("limelight-two");

            double rollAngleDifferenceOne = Math.abs(rollAngleOne - GyroSubsystem.getRollAngle().getDegrees());
            double rollAngleDifferenceTwo = Math.abs(rollAngleTwo - GyroSubsystem.getRollAngle().getDegrees());

            if (rollAngleDifferenceOne < rollAngleDifferenceTwo) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-one");
            } else if (rollAngleDifferenceTwo < rollAngleDifferenceOne) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-two");
            }

            break;
          case "CLOSEST_PITCH_ANGLE":
            double pitchAngleOne = getRedBotPosePitch("limelight-one");
            double pitchAngleTwo = getRedBotPosePitch("limelight-two");

            double pitchAngleDifferenceOne = Math.abs(pitchAngleOne - GyroSubsystem.getPitchAngle().getDegrees());
            double pitchAngleDifferenceTwo = Math.abs(pitchAngleTwo - GyroSubsystem.getPitchAngle().getDegrees());

            if (pitchAngleDifferenceOne < pitchAngleDifferenceTwo) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-one");
            } else if (pitchAngleDifferenceTwo < pitchAngleDifferenceOne) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-two");
            }
            break;
          case "AVERAGE_POSE":
            double xPoseOne = getRedBotPoseX("limelight-one");
            double yPoseOne = getRedBotPoseY("limelight-one");
            yawAngleOne = getRedBotPoseYaw("limelight-one");

            double xPoseTwo = getRedBotPoseX("limelight-two");
            double yPoseTwo = getRedBotPoseY("limelight-two");
            yawAngleTwo = getRedBotPoseYaw("limelight-two");

            double averageXPose = (xPoseOne + xPoseTwo) / 2;
            double averageYPose = (yPoseOne + yPoseTwo) / 2;
            double averageYawAngle = (yawAngleOne + yawAngleTwo) / 2;

            bestCameraPose = new Pose2d(averageXPose, averageYPose, new Rotation2d(averageYawAngle));

            break;
          // TODO find pose ambiguity values
          // case "WEIGHTED_AVERAGE_POSE":

          // break;
        }
      } else if (isTargetDetected("limelight-one")) {
        bestCameraPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-one");
      } else {
        bestCameraPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-two");
      }
    } else if (DriverStation.getAlliance().equals(Alliance.Blue)) {
      if (isTargetDetected("limelight-one") && isTargetDetected("limelight-two")) {
        switch (cameraFilterChooser.getSelected().name()) {
          case "CLOSEST_Z_POSE":
            double zPoseOne = getBlueBotPoseZ("limelight-one");
            double zPoseTwo = getBlueBotPoseZ("limelight-two");

            if (zPoseOne < zPoseTwo && zPoseOne > 0) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-one");
            } else if (zPoseTwo < zPoseOne && zPoseTwo > 0) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
            }

            break;
          case "CLOSEST_YAW_ANGLE":
            double yawAngleOne = getBlueBotPoseYaw("limelight-one");
            double yawAngleTwo = getBlueBotPoseYaw("limelight-two");

            double yawAngleDifferenceOne = Math.abs(yawAngleOne - GyroSubsystem.getYawAngle().getDegrees());
            double yawAngleDifferenceTwo = Math.abs(yawAngleTwo - GyroSubsystem.getYawAngle().getDegrees());

            if (yawAngleDifferenceOne < yawAngleDifferenceTwo) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-one");
            } else if (yawAngleDifferenceTwo < yawAngleDifferenceOne) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
            }

            break;
          case "CLOSEST_ROLL_ANGLE":
            double rollAngleOne = getBlueBotPoseRoll("limelight-one");
            double rollAngleTwo = getBlueBotPoseRoll("limelight-two");

            double rollAngleDifferenceOne = Math.abs(rollAngleOne - GyroSubsystem.getRollAngle().getDegrees());
            double rollAngleDifferenceTwo = Math.abs(rollAngleTwo - GyroSubsystem.getRollAngle().getDegrees());

            if (rollAngleDifferenceOne < rollAngleDifferenceTwo) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-one");
            } else if (rollAngleDifferenceTwo < rollAngleDifferenceOne) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
            }

            break;
          case "CLOSEST_PITCH_ANGLE":
            double pitchAngleOne = getBlueBotPosePitch("limelight-one");
            double pitchAngleTwo = getBlueBotPosePitch("limelight-two");

            double pitchAngleDifferenceOne = Math.abs(pitchAngleOne - GyroSubsystem.getPitchAngle().getDegrees());
            double pitchAngleDifferenceTwo = Math.abs(pitchAngleTwo - GyroSubsystem.getPitchAngle().getDegrees());

            if (pitchAngleDifferenceOne < pitchAngleDifferenceTwo) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-one");
            } else if (pitchAngleDifferenceTwo < pitchAngleDifferenceOne) {
              bestCameraPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
            }

            break;
          case "AVERAGE_POSE":
            double xPoseOne = getBlueBotPoseX("limelight-one");
            double yPoseOne = getBlueBotPoseY("limelight-one");
            yawAngleOne = getBlueBotPoseYaw("limelight-one");

            double xPoseTwo = getBlueBotPoseX("limelight-two");
            double yPoseTwo = getBlueBotPoseY("limelight-two");
            yawAngleTwo = getBlueBotPoseYaw("limelight-two");

            double averageXPose = (xPoseOne + xPoseTwo) / 2;
            double averageYPose = (yPoseOne + yPoseTwo) / 2;
            double averageYawAngle = (yawAngleOne + yawAngleTwo) / 2;

            bestCameraPose = new Pose2d(averageXPose, averageYPose, new Rotation2d(averageYawAngle));

            break;
          //TODO find pose ambiguity values
          // case "WEIGHTED_AVERAGE_POSE":

          // break;
        }
      } else if (isTargetDetected("limelight-one")) {
        bestCameraPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-one");
      } else {
        bestCameraPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
      }
    }

    return bestCameraPose;
  }

  // APRILTAG GET METHODS \\

  public static double getAprilTagID(String limelightHostName) {
    return LimelightHelpers.getFiducialID(limelightHostName);
  }

  /**
   * Returns an array of the [x (meters), y (meters), z (meters), roll (degrees),
   * pitch (degrees), yaw (degrees)]
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

  public static double getBotPoseLatency(String limelightHostName) {
    double botPoseArray[] = get3dBotPose(limelightHostName);
    return botPoseArray[6];
  }

  /**
   * Gets the robot's position in 3d space relative to the red alliance driver
   * stations
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

  public static double getRedBotPoseLatency(String limelightHostName) {
    double botPoseArray[] = getRed3dBotPose(limelightHostName);
    return botPoseArray[6];
  }

  /**
   * Gets the robot's position in 3d space relative to the blue alliance driver
   * stations
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

  public static double getBlueBotPoseLatency(String limelightHostName) {
    double botPoseArray[] = getBlue3dBotPose(limelightHostName);
    return botPoseArray[6];
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

  /**
   * Works for retroreflective, cube, and cone (depending on which pipeline is
   * set)
   */
  public static double getX(String limelightHostName) {
    return LimelightHelpers.getTX(limelightHostName);
  }

  /**
   * Works for retroreflective, cube, and cone (depending on which pipeline is
   * set)
   */
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
