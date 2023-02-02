// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {

  private ElevatorSubsystem elevatorSubsystem;

  private double position;

  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;

    position = pos;

    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
      elevatorSubsystem.setElevatorPosition(position);

      if(position < elevatorSubsystem.getElelvatorPositionInMeters())
      {          
        elevatorSubsystem.goToPosition(-ElevatorConstants.ELEVATOR_FF);
      }
      else
      {         
        elevatorSubsystem.goToPosition(ElevatorConstants.ELEVATOR_FF);
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
