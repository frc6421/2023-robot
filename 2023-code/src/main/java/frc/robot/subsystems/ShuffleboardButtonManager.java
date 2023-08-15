// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GridStates;
import frc.robot.Constants.LEDStates;
import frc.robot.Constants.RobotStates;

public class ShuffleboardButtonManager extends SubsystemBase {
  public ShuffleboardTab CompetitionTab = Shuffleboard.getTab("Competition");

  private ShuffleboardButtons curButton;

  private GenericEntry yellowButton;
  private GenericEntry purpleButton;

  private GenericEntry gridOneButton;
  private GenericEntry gridTwoButton;
  private GenericEntry gridThreeButton;

  private LEDStates ledState = LEDStates.YELLOW;

  // All of the shuffle buttons created
  ShuffleboardButtons[] buttons = {
      new ShuffleboardButtons("L High", RobotStates.HIGH_LEFT, 0, 0),
      new ShuffleboardButtons("L Mid", RobotStates.MID_LEFT, 0, 1),
      new ShuffleboardButtons("L Hybrid", RobotStates.HYBRID_LEFT, 0, 2),
      new ShuffleboardButtons("C High", RobotStates.HIGH_CENTER, 1, 0),
      new ShuffleboardButtons("C Mid", RobotStates.MID_CENTER, 1, 1),
      new ShuffleboardButtons("C Hybrid", RobotStates.HYBRID_CENTER, 1, 2),
      new ShuffleboardButtons("R High", RobotStates.HIGH_RIGHT, 2, 0),
      new ShuffleboardButtons("R Mid", RobotStates.MID_RIGHT, 2, 1),
      new ShuffleboardButtons("R Hybrid", RobotStates.HYBRID_RIGHT, 2, 2),
      new ShuffleboardButtons("L Substation", RobotStates.LEFT_SUBSTATION, 4, 0),
      new ShuffleboardButtons("R Substation", RobotStates.RIGHT_SUBSTATION, 5, 0),
      new ShuffleboardButtons("Single Sub", RobotStates.SINGLE_SUBSTATION, 6, 0)
    };

  /** Creates a new ShuffleboardButtonManager. */
  public ShuffleboardButtonManager() {
    curButton = buttons[0];
    curButton.setValue(true);
    RobotContainer.robotState = curButton.getRobotState();

    yellowButton = CompetitionTab.add("Yellow", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(4, 2)
    .getEntry();

    purpleButton = CompetitionTab.add("Purple", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(5, 2)
    .getEntry();

    gridOneButton = CompetitionTab.add("Grid 1", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(4, 1)
    .getEntry();

    gridTwoButton = CompetitionTab.add("Grid 2", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(5, 1)
    .getEntry();

    gridThreeButton = CompetitionTab.add("Grid 3", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(6, 1)
    .getEntry();

    CompetitionTab.addString("Robot State", () -> RobotContainer.robotState.name())
    .withPosition(4, 3);

    CompetitionTab.addString("LED State", () -> ledState.name())
    .withPosition(5, 3);

    CompetitionTab.addString("Grid State", () -> RobotContainer.gridState.name())
    .withPosition(6, 3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Checks if there are any other buttons that are set to true and sets all other
    // buttons to false
    for (ShuffleboardButtons button : buttons) {
      if (button.isActive() && !button.equals(curButton.getName())) {
        curButton.setValue(false);
        curButton = button;
        RobotContainer.robotState = curButton.getRobotState();
        break;
      }
    }

    // TESTING BUTTON FIX \\
    if(RobotContainer.robotState == RobotStates.DRIVE || RobotContainer.robotState == RobotStates.INTAKE) {
      curButton.setValue(false);
    }

    if(yellowButton.getBoolean(false) && ledState == LEDStates.PURPLE) {
      BlinkinSubsystem.blinkinYellowSet();
      purpleButton.setBoolean(false);
      ledState = LEDStates.YELLOW;
    }
    
    if(purpleButton.getBoolean(false) && ledState == LEDStates.YELLOW) {
      BlinkinSubsystem.blinkinVioletSet();
      yellowButton.setBoolean(false);
      ledState = LEDStates.PURPLE;
    }

    if(gridOneButton.getBoolean(false) && RobotContainer.gridState != GridStates.ONE) {
      gridTwoButton.setBoolean(false);
      gridThreeButton.setBoolean(false);
      RobotContainer.gridState = GridStates.ONE;
    }

    if(gridTwoButton.getBoolean(false) && RobotContainer.gridState != GridStates.TWO) {
      gridOneButton.setBoolean(false);
      gridThreeButton.setBoolean(false);
      RobotContainer.gridState = GridStates.TWO;
    }

    if(gridThreeButton.getBoolean(false) && RobotContainer.gridState != GridStates.THREE) {
      gridOneButton.setBoolean(false);
      gridTwoButton.setBoolean(false);
      RobotContainer.gridState = GridStates.THREE;
    }
  }

  // Returns the current buttons name
  public String getCurButtonName() {
    return curButton.getName();
  }
}
