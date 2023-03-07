// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ArmCommand.PlaceStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDownCommand extends CommandBase {
  /** Creates a new IntakeUpCommand. */
  IntakeSubsystem intake;
  IntakeArmCommand intakeArmCommand;
  Timer timer;

  public IntakeDownCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeSubsystem;
    
    timer = new Timer();

    // addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.setIntakeSpeed(1);
    intakeArmCommand = new IntakeArmCommand(intake, IntakeArmCommand.IntakePlaceStates.FLOOR);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intake.setIntakeSpeed(0.2);
    intakeArmCommand = new IntakeArmCommand(intake, IntakeArmCommand.IntakePlaceStates.DRIVE);
    timer.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intake.getIntakeVelocity() < 200) && (timer.get() > 0.5);
  }
}
