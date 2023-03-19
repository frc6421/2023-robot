// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.ArmAngleConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmElevatorCommand. */
  ArmSubsystem arm;
  Timer timer = new Timer();

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(600, 600);
  private TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State();
  
  TrapezoidProfile armProfile;

  public ArmCommand(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(armSubsystem);

      arm = armSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    timer.reset();

    switch(RobotContainer.robotState)
    {
      case DRIVE:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.ARM_DRIVE_ANGLE, 0);
        break;

      case INTAKE:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.ARM_INTAKE_ANGLE, 0);
        break;

      case HYBRID:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.ARM_HYBRID_ANGLE, 0);
        break;

      case HIGH_LEFT:
      case HIGH_CENTER:
      case HIGH_RIGHT:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.ARM_CONE_HIGH_ANGLE, 0);
        break;

      case MID_LEFT:
      case MID_CENTER:
      case MID_RIGHT:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.ARM_CONE_MID_ANGLE, 0);
        break;

      case LEFT_SUBSTATION:
      case RIGHT_SUBSTATION:
        armGoal = new TrapezoidProfile.State(ArmAngleConstants.ARM_SUBSTATION_ANGLE, 0);
        break;

    }

    armProfile = new TrapezoidProfile(armConstraints, armGoal, new TrapezoidProfile.State(arm.getArmDegreePosition(), 0));

    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    armSetpoint = armProfile.calculate(timer.get());
    
    arm.setPosition(armSetpoint.position);
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
