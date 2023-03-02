// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ArmDownThenReleaseCommand extends CommandBase {
  ArmSubsystem arm;

  GrabberSubsystem grab;

  Timer timer = new Timer();

  

    private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(100, 100);
      private TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
      private TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State();

      TrapezoidProfile armProfile;
      TrapezoidProfile elevatorProfile;
  
  /** Creates a new ArmDownthenReleaseCommand. */
  public ArmDownThenReleaseCommand(ArmSubsystem armSubsystem, GrabberSubsystem grabber) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(armSubsystem,grabber);

    arm = armSubsystem;

    grab = grabber;
    
      
    arm.setSetPoint(arm.getArmDegreePosition());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();

    armGoal = new TrapezoidProfile.State(arm.getArmDegreePosition() + 5, 0);

    armProfile = new TrapezoidProfile(armConstraints, armGoal, new TrapezoidProfile.State(arm.getArmDegreePosition(), 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSetpoint = armProfile.calculate(timer.get());

    arm.setPosition(armSetpoint.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setPosition(armSetpoint.position);
    arm.setSetPoint(armSetpoint.position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((timer.get() > armProfile.totalTime()));
  }
}
