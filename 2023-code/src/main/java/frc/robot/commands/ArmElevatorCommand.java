// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
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
      elevator = elevatorSubsystem;
      arm = armSubsystem;
      placeState = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_FORWARD_SOFT_LIMIT_METERS);
    arm.setArmAngleWithGrav(ArmConstants.CONE_HIGH_TOP_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    elevator.setElevatorPosition();
    arm.setArmAngleWithGrav(ArmConstants.ARM_IN_SOFT_LIMIT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
