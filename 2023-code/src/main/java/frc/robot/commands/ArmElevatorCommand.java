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

public class ArmElevatorCommand extends CommandBase {
  /** Creates a new ArmElevatorCommand. */
  ElevatorSubsystem elevator;
  ArmSubsystem arm;
  Timer timer = new Timer();
  

  private final TrapezoidProfile.Constraints elevatorConstraints =
      new TrapezoidProfile.Constraints(1, 0.5);
  private TrapezoidProfile.State elevatorGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State elevatorSetpoint = new TrapezoidProfile.State();

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(270, 270);
  private TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State();
  
  TrapezoidProfile armProfile;
  TrapezoidProfile elevatorProfile;
  


  public enum PlaceStates {
    FLOOR,
    INTAKE,
    MID,
    HIGH,
    SUBSTATION,
    UP
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
    timer.reset();
    timer.start();

    switch(placeState)
    {
      case FLOOR:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.FLOOR_ANGLE, 0);
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MIN_POS_IN, 0);
        
        break;

      case INTAKE:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.GRAB_FROM_INTAKE_ANGLE, 0);
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MIN_POS_IN, 0);
        break;

      case MID:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.CONE_MID_TOP_ANGLE, 0);
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MIN_POS_IN, 0);
        break;

      case HIGH:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.CONE_HIGH_TOP_ANGLE, 0);
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_FORWARD_SOFT_LIMIT_METERS, 0);
        break;

      case SUBSTATION:
        armGoal = new TrapezoidProfile.State(0, 0);
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MIN_POS_IN, 0);
        break;
      
      case UP:
        armGoal = new TrapezoidProfile.State(90, 0);
        elevatorGoal = new TrapezoidProfile.State(ElevatorConstants.ELEVATOR_MIN_POS_IN, 0);
        break;
    }

    armProfile = new TrapezoidProfile(armConstraints, armGoal, new TrapezoidProfile.State(arm.getArmDegreePosition(), 0));
    elevatorProfile = new TrapezoidProfile(elevatorConstraints, elevatorGoal, new TrapezoidProfile.State(elevator.getElelvatorPositionInMeters(), 0));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    armSetpoint = armProfile.calculate(timer.get());
    elevatorSetpoint = elevatorProfile.calculate(timer.get());
    
    elevator.setElevatorPosition(elevatorSetpoint.position);
    arm.setPosition(armSetpoint.position);
    SmartDashboard.putNumber("Arm Goal", armSetpoint.position);
    SmartDashboard.putNumber("Elevator Goal", elevatorSetpoint.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    arm.setPosition(armSetpoint.position);
    arm.setSetPoint(armSetpoint.position);
    elevator.setElevatorPosition(elevatorSetpoint.position);
    elevator.setSetPoint(elevatorSetpoint.position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((timer.get() > armProfile.totalTime()) && (timer.get() > elevatorProfile.totalTime()));
  }
}
