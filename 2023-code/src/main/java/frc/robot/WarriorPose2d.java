// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class WarriorPose2d extends Pose2d{
   /**
   * Multiplies the current y value of the pose by a scalar.
   *
   * @param scalar The scalar.
   * @return The new scaled Pose2d.
   */
  public Pose2d timesY(double scalar) {
    return new Pose2d(getX(), getY() * scalar, getRotation());
  }
}
