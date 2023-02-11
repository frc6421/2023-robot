// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants.ArmAngleConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmElevatorCommand extends CommandBase {
  /** Creates a new ArmElevatorCommand. */
  ElevatorSubsystem elevator;
  ArmSubsystem arm;

  public enum PlaceStates {
    FLOOR,
    INTAKE,
    MID,
    HIGH,
    SUBSTATION
  }
  private PlaceStates placeState;

  public ArmElevatorCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, PlaceStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(elevatorSubsystem, armSubsystem);
      placeState = state;

      elevator = elevatorSubsystem;
      arm = armSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    switch(placeState)
    {
      case FLOOR:
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_REVERSE_SOFT_LIMIT);
        arm.setArmAngleWithGrav(ArmAngleConstants.FLOOR_ANGLE);
        break;

      case INTAKE:
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_REVERSE_SOFT_LIMIT);
        arm.setArmAngleWithGrav(ArmAngleConstants.GRAB_FROM_INTAKE_ANGLE);
        break;

      case MID:
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_REVERSE_SOFT_LIMIT);
        arm.setArmAngleWithGrav(ArmAngleConstants.CONE_MID_TOP_ANGLE);
        break;

      case HIGH:
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_FORWARD_SOFT_LIMIT_METERS);
        arm.setArmAngleWithGrav(ArmAngleConstants.CONE_HIGH_TOP_ANGLE);
        break;

      case SUBSTATION:
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_FORWARD_SOFT_LIMIT_METERS);
        arm.setArmAngleWithGrav(0); // TODO: Find these values
        break;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
