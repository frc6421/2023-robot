// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants.WristAngleConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
  WristSubsystem wristSubsystem;
  Timer timer = new Timer();

  private final TrapezoidProfile.Constraints wristConstraints =
      new TrapezoidProfile.Constraints(600, 300);

  private TrapezoidProfile.State wristGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State wristSetpoint = new TrapezoidProfile.State();
  
  TrapezoidProfile wristProfile;

  /** Creates a new WristCommand. */
  public WristCommand(WristSubsystem wrist) {
    wristSubsystem = wrist;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();

    switch(RobotContainer.robotState)
    {
      case DRIVE:
        wristGoal = new TrapezoidProfile.State(WristAngleConstants.WRIST_DRIVE_ANGLE, 0);
        break;

      case INTAKE:
        wristGoal = new TrapezoidProfile.State(WristAngleConstants.WRIST_INTAKE_ANGLE, 0);
        break;

      case HYBRID:
        wristGoal = new TrapezoidProfile.State(WristAngleConstants.WRIST_HYBRID_ANGLE, 0);
        break;

      case HIGH_LEFT:
      case HIGH_CENTER:
      case HIGH_RIGHT:
        wristGoal = new TrapezoidProfile.State(WristAngleConstants.WRIST_CONE_HIGH_ANGLE, 0);
        break;

      case MID_LEFT:
      case MID_CENTER:
      case MID_RIGHT:
        wristGoal = new TrapezoidProfile.State(WristAngleConstants.WRIST_CONE_MID_ANGLE, 0);
        break;

      case LEFT_SUBSTATION:
      case RIGHT_SUBSTATION:
        wristGoal = new TrapezoidProfile.State(WristAngleConstants.WRIST_SUBSTATION_ANGLE, 0);
        break;

    }

    wristProfile = new TrapezoidProfile(wristConstraints, wristGoal, new TrapezoidProfile.State(wristSubsystem.getWristDegreePosition(), 0));

    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristSetpoint = wristProfile.calculate(timer.get());
    
    wristSubsystem.setPosition(wristSetpoint.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setPosition(wristSetpoint.position);
    wristSubsystem.setSetPoint(wristSetpoint.position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > wristProfile.totalTime();
  }
}
