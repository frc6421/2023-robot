// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class WarriorTranslation2d extends Translation2d {
    /**
   * Returns the y coordinates of the translation multiplied by a scalar.
   *
   * <p>For example, Translation2d(2.0, 2.5) * 2 = Translation2d(2.0, 5.0).
   *
   * @param scalar The scalar to multiply by.
   * @return The scaled translation.
   */
  public Translation2d timesY(double scalar) {
    return new Translation2d(getX(), getY() * scalar);
  }
}
