// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants.ArmAngleConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
  /** Creates a new ArmElevatorCommand. */
  ElevatorSubsystem elevator;
  Timer timer = new Timer();

  private final TrapezoidProfile.Constraints elevatorConstraints =
      new TrapezoidProfile.Constraints(8, 4);
  private TrapezoidProfile.State elevatorGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State elevatorSetpoint = new TrapezoidProfile.State();
  
  TrapezoidProfile elevatorProfile;
  
  public enum PlaceStates {
    FLOOR,
    MID,
    HIGH,
    SUBSTATION,
    HYBRID, 
    TRANSFER,
    DRIVE
  }
  private PlaceStates placeState;

  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, PlaceStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(elevatorSubsystem);
      placeState = state;
      
      elevator = elevatorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer.reset();
    timer.start();

    switch(placeState)
    {
      case FLOOR:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MIN_POS_IN, 0);
        
        break;

      case MID:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MIN_POS_IN, 0);
        break;

      case HIGH:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_FORWARD_SOFT_LIMIT_METERS, 0);
        break;

      case SUBSTATION:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_SUBSTATION_LENGTH, 0);
        break;

      case HYBRID:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MIN_POS_IN, 0);
        break;

      case TRANSFER:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_FORWARD_SOFT_LIMIT_METERS, 0);
        break;

      case DRIVE:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MIN_POS_IN, 0);
        break;
    }

    elevatorProfile = new TrapezoidProfile(elevatorConstraints, elevatorGoal, new TrapezoidProfile.State(ElevatorSubsystem.getElevatorEncoderPosition(), 0));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 

  {
    elevatorSetpoint = elevatorProfile.calculate(timer.get());
    
    elevator.setElevatorPosition(elevatorSetpoint.position);
    SmartDashboard.putNumber("Elevator Goal", elevatorSetpoint.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    elevator.setElevatorPosition(elevatorSetpoint.position);
    elevator.setSetPoint(elevatorSetpoint.position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > elevatorProfile.totalTime());
  }
}
