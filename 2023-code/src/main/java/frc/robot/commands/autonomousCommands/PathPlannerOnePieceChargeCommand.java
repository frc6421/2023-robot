// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ArmElevatorCommand;
import frc.robot.commands.ArmElevatorCommand.PlaceStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerOnePieceChargeCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ArmSubsystem armSubsystem;
  private GrabberSubsystem grabberSubsystem;
  /** Creates a new PathPlannerOnePieceChargeCommand. */
  public PathPlannerOnePieceChargeCommand(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm, GrabberSubsystem grabber) {
    driveSubsystem = drive;
    elevatorSubsystem = elevator;
    armSubsystem = arm;
    grabberSubsystem = grabber;
    addRequirements(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);
    
    PathPlannerTrajectory onePieceChargeTrajectory = PathPlanner.loadPath("OnePieceChargePath", new PathConstraints(AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND, AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    PPSwerveControllerCommand onePieceChargeCommand = new PPSwerveControllerCommand(
            onePieceChargeTrajectory, 
            driveSubsystem::getPose2d, // Pose supplier
            driveSubsystem.swerveKinematics, // SwerveDriveKinematics
            new PIDController(AutoConstants.X_DRIVE_P, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(AutoConstants.Y_DRIVE_P, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(AutoConstants.THETA_P, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            driveSubsystem::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveSubsystem // Requires this drive subsystem
        );

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.resetOdometry(onePieceChargeTrajectory.getInitialHolonomicPose())),
      onePieceChargeCommand,
      new InstantCommand(() -> driveSubsystem.visionDrive(0, 0, 0))
    );
  }
}
