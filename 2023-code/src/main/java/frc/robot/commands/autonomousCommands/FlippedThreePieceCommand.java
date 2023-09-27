// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlippedThreePieceCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private WristSubsystem wristSubsystem;
  private ArmSubsystem armSubsystem;
  private ElevatorSubsystem elevatorSubsystem;

  /** Creates a new FlippedThreePieceCommand. */
  public FlippedThreePieceCommand(DriveSubsystem drive, IntakeSubsystem intake, WristSubsystem wrist, ArmSubsystem arm, ElevatorSubsystem elevator) {
    driveSubsystem = drive;
    intakeSubsystem = intake;
    wristSubsystem = wrist;
    armSubsystem = arm;
    elevatorSubsystem = elevator;

    addRequirements(driveSubsystem, intakeSubsystem, wristSubsystem, armSubsystem, elevatorSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND + 0.5,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics);

    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND + 0.5,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED + 1.5)
        .setKinematics(driveSubsystem.swerveKinematics)
        .setReversed(true);

    // First action is scoring pre-loaded cone on high, second cone node

    // Pick up cube
    Trajectory pickUpTrajectoryOne = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FLIPPED_SECOND_CONE_NODE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FLIPPED_FOURTH_GAME_PIECE.plus(new Translation2d(Units.feetToMeters(0), 0)), new Rotation2d(0))
      ), forwardConfig);

    // Score cube (high)
    Trajectory scoreTrajectoryOne = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FLIPPED_FOURTH_GAME_PIECE.plus(new Translation2d(Units.feetToMeters(0), 0)), new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FLIPPED_FAR_EDGE_OF_COMMUNITY, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FLIPPED_AROUND_CHARGE_STATION, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FLIPPED_CUBE_NODE.plus(new Translation2d(Units.feetToMeters(1), -Units.feetToMeters(.5))), new Rotation2d(0))
    ), reverseConfig);

    // Pick up second cone
    Trajectory pickUpTrajectoryTwo = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FLIPPED_CUBE_NODE.plus(new Translation2d(Units.feetToMeters(1), -Units.feetToMeters(.5))), new Rotation2d(0)),
      new Pose2d(TrajectoryConstants.FLIPPED_AROUND_CHARGE_STATION, new Rotation2d(0)),
      new Pose2d(TrajectoryConstants.FLIPPED_FAR_EDGE_OF_COMMUNITY, new Rotation2d(0)),
      new Pose2d(TrajectoryConstants.FLIPPED_FAR_EDGE_OF_COMMUNITY.plus(new Translation2d(Units.feetToMeters(2), -Units.feetToMeters(4.8))), new Rotation2d(0)),
      new Pose2d(TrajectoryConstants.FLIPPED_THIRD_GAME_PIECE.plus(new Translation2d(0, -Units.feetToMeters(0.5))), new Rotation2d(0))
    ), forwardConfig);

    // Score third piece (cone)
    Trajectory scoreTrajectoryTwo = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FLIPPED_THIRD_GAME_PIECE.plus(new Translation2d(0, -Units.feetToMeters(0.5))), new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.FLIPPED_FAR_EDGE_OF_COMMUNITY, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.FLIPPED_AROUND_CHARGE_STATION, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FLIPPED_FIRST_CONE_NODE.plus(new Translation2d(Units.feetToMeters(1), -Units.feetToMeters(2))), new Rotation2d(Units.degreesToRadians(0)))), 
        reverseConfig);

    var thetaController = new ProfiledPIDController(
          AutoConstants.THETA_P, AutoConstants.THETA_I, AutoConstants.THETA_D,
          new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
              AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
          // Position controllers
          new PIDController(AutoConstants.X_DRIVE_P, AutoConstants.X_DRIVE_I, AutoConstants.X_DRIVE_D),
          new PIDController(AutoConstants.Y_DRIVE_P, AutoConstants.Y_DRIVE_I, AutoConstants.Y_DRIVE_D),
          thetaController);

      SwerveControllerCommand pickUpOneCommand = new SwerveControllerCommand(
          pickUpTrajectoryOne,
          driveSubsystem::getPose2d,
          driveSubsystem.swerveKinematics,
          holonomicDriveController,
          driveSubsystem::setModuleStates,
          driveSubsystem);

      SwerveControllerCommand scoreOneCommand = new SwerveControllerCommand(
          scoreTrajectoryOne,
          driveSubsystem::getPose2d,
          driveSubsystem.swerveKinematics,
          holonomicDriveController,
          driveSubsystem::setModuleStates,
          driveSubsystem);

      SwerveControllerCommand pickUpTwoCommand = new SwerveControllerCommand(
          pickUpTrajectoryTwo,
          driveSubsystem::getPose2d,
          driveSubsystem.swerveKinematics,
          holonomicDriveController,
          driveSubsystem::setModuleStates,
          driveSubsystem);

      SwerveControllerCommand scoreTwoCommand = new SwerveControllerCommand(
          scoreTrajectoryTwo,
          driveSubsystem::getPose2d,
          driveSubsystem.swerveKinematics,
          holonomicDriveController,
          driveSubsystem::setModuleStates,
          driveSubsystem);
      
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.resetOdometry(pickUpTrajectoryOne.getInitialPose())),
      // Mid score cone 1
      new InstantCommand(() -> RobotContainer.robotState = RobotStates.MID_LEFT),
        new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)),
      new ParallelDeadlineGroup(new WaitCommand(0.14), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED))),
      // Drive to intake cube
      new InstantCommand(() -> RobotContainer.robotState = RobotStates.AUTO_INTAKE),
      new ParallelDeadlineGroup(pickUpOneCommand, 
                      new SequentialCommandGroup(new WaitCommand(0), 
                              new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_PICK_UP_SPEED))))),
      new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
      new InstantCommand(() -> RobotContainer.robotState = RobotStates.DRIVE),
      new ParallelCommandGroup(scoreOneCommand, new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER))),
      new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
      // Mid score cube
      new InstantCommand(() -> RobotContainer.robotState = RobotStates.MID_LEFT),
        new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)),
      new ParallelDeadlineGroup(new WaitCommand(0.14), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED))),
      // Drive to intake cone 2
      new InstantCommand(() -> RobotContainer.robotState = RobotStates.AUTO_INTAKE),
      new ParallelDeadlineGroup(pickUpTwoCommand, 
                      new SequentialCommandGroup(new WaitCommand(0), 
                              new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_PICK_UP_SPEED))))),
      new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
      new InstantCommand(() -> RobotContainer.robotState = RobotStates.DRIVE),
      new ParallelCommandGroup(scoreTwoCommand, new WristCommand(wristSubsystem), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER))),
      new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
      // High score cone 2
      new InstantCommand(() -> RobotContainer.robotState = RobotStates.MID_LEFT),
      new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)),
      new ParallelDeadlineGroup(new WaitCommand(0.14), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED))),
      // Set to drive position
      new InstantCommand(() -> RobotContainer.robotState = RobotStates.DRIVE),
      new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED)))
    );
  }
}
