// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDStates;
import frc.robot.Constants.RobotStates;

public class ShuffleboardButtonManager extends SubsystemBase {
  public ShuffleboardTab CompetitionTab = Shuffleboard.getTab("Competition");

  private ShuffleboardButtons curButton;

  private GenericEntry yellowButton;
  private GenericEntry purpleButton;

  private LEDStates ledState = LEDStates.YELLOW;

  // All of the shuffle buttons created
  ShuffleboardButtons[] buttons = {
      new ShuffleboardButtons("L High", RobotStates.HIGH_LEFT, 0, 0),
      new ShuffleboardButtons("L Mid", RobotStates.MID_LEFT, 0, 1),
      new ShuffleboardButtons("L Hybrid", RobotStates.HYBRID, 0, 2),
      new ShuffleboardButtons("C High", RobotStates.HIGH_CENTER, 1, 0),
      new ShuffleboardButtons("C Mid", RobotStates.MID_CENTER, 1, 1),
      new ShuffleboardButtons("C Hybrid", RobotStates.HYBRID, 1, 2),
      new ShuffleboardButtons("R High", RobotStates.HIGH_RIGHT, 2, 0),
      new ShuffleboardButtons("R Mid", RobotStates.MID_RIGHT, 2, 1),
      new ShuffleboardButtons("R Hybrid", RobotStates.HYBRID, 2, 2),
      new ShuffleboardButtons("L Substation", RobotStates.LEFT_SUBSTATION, 4, 0),
      new ShuffleboardButtons("R Substation", RobotStates.RIGHT_SUBSTATION, 5, 0),
  };

  /** Creates a new ShuffleboardButtonManager. */
  public ShuffleboardButtonManager() {
    curButton = buttons[0];

    yellowButton = CompetitionTab.add("Yellow", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(4, 2)
    .getEntry();

    purpleButton = CompetitionTab.add("Purple", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(5, 2)
    .getEntry();

    CompetitionTab.addString("Robot State", () -> RobotContainer.robotState.name())
    .withPosition(4, 5);

    CompetitionTab.addString("LED State", () -> ledState.name())
    .withPosition(5, 5);
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
  }

  // Returns the current buttons name
  public String getCurButtonName() {
    return curButton.getName();
  }
}
