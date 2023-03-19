// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.RobotStates;

/** Add your docs here. */
public class ShuffleboardButtons {
     /** Creates a new SuffleBoardButtons. */
  public ShuffleboardTab CompetitionTab = Shuffleboard.getTab("Competition");
  
  private boolean isButtonActive;

  private GenericEntry button;

  private String name;

  private RobotStates robotState;

  public ShuffleboardButtons(String name, RobotStates robotState, int col, int row) {
    // Inital values being set
    isButtonActive = false;

    this.name = name;
    this.robotState = robotState;

    // Creates a new grid button
    button = CompetitionTab.add(name, isButtonActive)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(col, row)
    .getEntry();
  }

  // Sets the buttons boolean
  public void setValue(boolean val)
  {
    button.setBoolean(val);
  }

  // Returns the current boolean value of the button
  public boolean isActive()
  {
    return button.getBoolean(false);
  }

  public RobotStates getRobotState() {
    return this.robotState;
  }
  
  // Returns the name
  public String getName()
  {
    return name;
  }

  /**
   * Checks to see two grid buttons are equal based on the names of the buttons
   * @param name the name of a grid button
   * @return true if equal and false if not equal
   */
  public boolean equals(String name) {
    return this.name.equals(name);
  }
}
