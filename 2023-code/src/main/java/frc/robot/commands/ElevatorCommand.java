// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
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

  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(elevatorSubsystem);
      
      elevator = elevatorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer.reset();

    switch(RobotContainer.robotState)
    {
      case DRIVE:
      case INTAKE:
      case HYBRID_LEFT:
      case HYBRID_CENTER:
      case HYBRID_RIGHT:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MIN_POS_IN, 0);

        break;
      case MID_LEFT:
      case MID_CENTER:
      case MID_RIGHT:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MID_POSITION, 0);
        
        break;

      case HIGH_LEFT:
      case HIGH_CENTER:
      case HIGH_RIGHT:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_HIGH_POSITION, 0);

        break;
      
      case LEFT_SUBSTATION:
      case RIGHT_SUBSTATION:
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_SUBSTATION_POSITION, 0);

        break;
    }

    elevatorProfile = new TrapezoidProfile(elevatorConstraints, elevatorGoal, new TrapezoidProfile.State(ElevatorSubsystem.getElevatorEncoderPosition(), 0));

    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 

  {
    elevatorSetpoint = elevatorProfile.calculate(timer.get());
    
    elevator.setElevatorPosition(elevatorSetpoint.position);
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
