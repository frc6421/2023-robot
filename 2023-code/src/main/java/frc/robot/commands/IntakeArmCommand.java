// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmCommand extends CommandBase {

  IntakeSubsystem intakeSubsystem;

  Timer timer = new Timer();

  private TrapezoidProfile.Constraints intakeArmConstraints =
      new TrapezoidProfile.Constraints(900, 900); // 1260, 1260
  private TrapezoidProfile.State intakeArmGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State intakeArmSetPoint = new TrapezoidProfile.State();

  TrapezoidProfile intakeArmProfile;

  public enum IntakePlaceStates{
    FLOOR,
    DRIVE, 
    HYBRID,
    SINGLE
  }

  private IntakePlaceStates intakePlaceStates;

  /** Creates a new IntakeArmCommand. */
  public IntakeArmCommand(IntakeSubsystem intakeSubsystem, IntakePlaceStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);

    intakePlaceStates = state;
  
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer.reset();
    timer.start();

    switch(intakePlaceStates)
    {
      case FLOOR:
      //intakeSubsystem.intakeArmMotor.setIdleMode(IdleMode.kBrake);
      intakeArmConstraints =
      new TrapezoidProfile.Constraints(900, 300); // 1260, 1260
      intakeArmGoal = new TrapezoidProfile.State();
      intakeArmSetPoint = new TrapezoidProfile.State();
      intakeArmGoal = new TrapezoidProfile.State(IntakeConstants.INTAKE_FLOOR_ANGLE, 0);


        break;
      case DRIVE:
      //intakeSubsystem.intakeArmMotor.setIdleMode(IdleMode.kCoast);
        intakeArmGoal = new TrapezoidProfile.State(IntakeConstants.INTAKE_DRIVE_ANGLE + 45, 0);
  
        break;
      case HYBRID:
        //intakeSubsystem.intakeArmMotor.setIdleMode(IdleMode.kBrake);
        intakeArmGoal = new TrapezoidProfile.State(IntakeConstants.INTAKE_HYBRID_ANGLE, 0);

        break;
      case SINGLE:
      //intakeSubsystem.intakeArmMotor.setIdleMode(IdleMode.kCoast);
        intakeArmGoal = new TrapezoidProfile.State(IntakeConstants.INTAKE_SINGLE_ANGLE, 0);
        
        break;
    }
    intakeArmProfile = new TrapezoidProfile(intakeArmConstraints, intakeArmGoal, new TrapezoidProfile.State(intakeSubsystem.getIntakeArmDegreePosition(), 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    intakeArmSetPoint = intakeArmProfile.calculate(timer.get());

    intakeSubsystem.setPosition(intakeArmSetPoint.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    intakeSubsystem.setPosition(intakeArmSetPoint.position);
    intakeSubsystem.setSetPoint(intakeArmSetPoint.position);

    intakeArmConstraints =
      new TrapezoidProfile.Constraints(900, 900); // 1260, 1260
      intakeArmGoal = new TrapezoidProfile.State();
      intakeArmSetPoint = new TrapezoidProfile.State();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > intakeArmProfile.totalTime()/* + 2.0 */);
  }
}
