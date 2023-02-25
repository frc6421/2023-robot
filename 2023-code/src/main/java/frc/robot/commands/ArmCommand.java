// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ArmAngleConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmElevatorCommand. */
  ArmSubsystem arm;
  Timer timer = new Timer();

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(100, 100);
  private TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State();
  
  TrapezoidProfile armProfile;
  
  public enum PlaceStates {
    FLOOR,
    INTAKE,
    MID,
    HIGH,
    SUBSTATION,
    UP,
    HYBRID, 
    TRANSFER
  }
  private PlaceStates placeState;

  public ArmCommand(ArmSubsystem armSubsystem, PlaceStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(armSubsystem);
      placeState = state;

      arm = armSubsystem;

      arm.setSetPoint(arm.getArmDegreePosition());
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
        break;

      case INTAKE:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.GRAB_FROM_INTAKE_ANGLE, 0);
        break;

      case MID:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.CONE_MID_TOP_ANGLE, 0);
        break;

      case HIGH:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.CONE_HIGH_TOP_ANGLE, 0);
        break;

      case SUBSTATION:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.GRAB_FROM_SUBSTATION_ANGLE, 0);
        break;
      
      case UP:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.UP_POSITION, 0);
        break;

      case HYBRID:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.GRAB_FROM_INTAKE_ANGLE, 0);
        break;

      case TRANSFER:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.TRANSFER_ANGLE, 0);
        break;

    }

    armProfile = new TrapezoidProfile(armConstraints, armGoal, new TrapezoidProfile.State(arm.getArmDegreePosition() - 6, 0));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 

  {
    armSetpoint = armProfile.calculate(timer.get());
    
    arm.setPosition(armSetpoint.position);
    SmartDashboard.putNumber("Arm Goal", armSetpoint.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    arm.setPosition(armSetpoint.position);
    arm.setSetPoint(armSetpoint.position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > armProfile.totalTime());
  }
}