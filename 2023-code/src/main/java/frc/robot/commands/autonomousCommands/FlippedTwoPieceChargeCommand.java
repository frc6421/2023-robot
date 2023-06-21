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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.AutoConstants.TrajectoryConstants;
import frc.robot.Constants.RobotStates;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
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
public class FlippedTwoPieceChargeCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ArmSubsystem armSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private WristSubsystem wristSubsystem;

  /** Creates a new FlippedTwoPieceChargeCommand. */
  public FlippedTwoPieceChargeCommand(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm, IntakeSubsystem intake, WristSubsystem wrist) {
    driveSubsystem = drive;
    elevatorSubsystem = elevator;
    armSubsystem = arm;
    intakeSubsystem = intake;
    wristSubsystem = wrist;
    addRequirements(driveSubsystem, elevatorSubsystem, armSubsystem, intakeSubsystem, wristSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics);

    TrajectoryConfig chargeConfig = new TrajectoryConfig(
      AutoConstants.AUTO_CHARGE_MAX_VELOCITY_METERS_PER_SECOND,
      AutoConstants.AUTO_CHARGE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
      .setKinematics(driveSubsystem.swerveKinematics);

    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.swerveKinematics)
        .setReversed(true);

    // Starts at far left cone node
    // Stage cube in far left game piece
    Trajectory firstPickUpTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FLIPPED_SECOND_CONE_NODE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FLIPPED_FOURTH_GAME_PIECE, new Rotation2d(0))), forwardConfig);

    // Return to score cube next to cone
    Trajectory firstScoreTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FLIPPED_FOURTH_GAME_PIECE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FLIPPED_CUBE_NODE, new Rotation2d(0))), reverseConfig);

    // Drives on charge station
    Trajectory chargeStationTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FLIPPED_CUBE_NODE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FLIPPED_COOPERTITION_CUBE_NODE.plus(new Translation2d(Units.inchesToMeters(48), 0)), new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FLIPPED_CENTER_OF_CHARGE_STATION.plus(new Translation2d(0, Units.inchesToMeters(21.39))), new Rotation2d(0))), chargeConfig);

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

    SwerveControllerCommand firstPickUpCommand = new SwerveControllerCommand(
        firstPickUpTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.swerveKinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand firstScoreCommand = new SwerveControllerCommand(
        firstScoreTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.swerveKinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand chargeStationCommand = new SwerveControllerCommand(
        chargeStationTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.swerveKinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> driveSubsystem.resetOdometry(firstPickUpTrajectory.getInitialPose())),
        new InstantCommand(() -> RobotContainer.robotState = RobotStates.HIGH_LEFT),
        new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)),
        new ParallelDeadlineGroup(new WaitCommand(0.3), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED))),
        new InstantCommand(() -> RobotContainer.robotState = RobotStates.DRIVE),
        new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)),
        new InstantCommand(() -> RobotContainer.robotState = RobotStates.INTAKE),
        new ParallelDeadlineGroup(firstPickUpCommand, 
                        new SequentialCommandGroup(new WaitCommand(0.2), 
                                new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_PICK_UP_SPEED))))),
        new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
        new InstantCommand(() -> RobotContainer.robotState = RobotStates.DRIVE),
        new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER))),
        firstScoreCommand,
        new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
        new InstantCommand(() -> RobotContainer.robotState = RobotStates.HIGH_CENTER),
        new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)),
        new ParallelDeadlineGroup(new WaitCommand(0.3), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED))),
        new InstantCommand(() -> RobotContainer.robotState = RobotStates.DRIVE),
        new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER))),
        chargeStationCommand,
        new InstantCommand(() -> driveSubsystem.autoDrive(0, 0, 0)),
        new BalanceCommand(driveSubsystem));
  }
}
