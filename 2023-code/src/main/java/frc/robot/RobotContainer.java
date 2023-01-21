// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem;

  private final ElevatorSubsystem elevatorSubsystem;

  // Set up controller with CommandXboxController
  private final CommandXboxController driverController;

  private ShuffleboardTab elevatorTab;
  private GenericEntry elevatorFFTestingEntry;
  private GenericEntry elevatorPositionTestEntry;
  private GenericEntry elevatorPTestingEntry;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();

    elevatorSubsystem = new ElevatorSubsystem();

    driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    SmartDashboard.putNumber("LeftY", driverController.getLeftY());
    SmartDashboard.putNumber("LeftX", driverController.getLeftX());
    SmartDashboard.putNumber("RightX", driverController.getRightX());

    driveSubsystem.setDefaultCommand(new RunCommand(() ->
      driveSubsystem.drive(
        driverController.getLeftY(),
        driverController.getLeftX(),
        driverController.getRightX()), driveSubsystem));
    
    elevatorSubsystem.setDefaultCommand(new RunCommand(() -> 
      elevatorSubsystem.setElevatorWithPercent(-driverController.getRightY()), elevatorSubsystem)
    );

    elevatorTab = Shuffleboard.getTab("Elevator Tab");
    elevatorFFTestingEntry = elevatorTab.add("Set Elevator FF: ", 0)
      .getEntry();
    elevatorPTestingEntry = elevatorTab.add("Set Elevator P: ", 0)
      .getEntry();
    elevatorFFTestingEntry = elevatorTab.add("Set Elevator Pos: ", 0)
      .getEntry();
    
    //Feed forward tuning
    //driverController.x().onTrue(new RunCommand(() -> elevatorSubsystem.setElevatorWithPercent(.02375), elevatorSubsystem));
    //driverController.x().onFalse(new RunCommand(() -> elevatorSubsystem.setElevatorWithPercent(0), elevatorSubsystem));

    driverController.x().whileTrue(new RunCommand(() -> elevatorSubsystem.setElevatorWithPercent(elevatorFFTestingEntry.getDouble(0)), elevatorSubsystem));
    driverController.y().whileTrue(new RunCommand(() -> elevatorSubsystem.setP(elevatorPTestingEntry.getDouble(0)), elevatorSubsystem));
    driverController.b().whileTrue(new RunCommand(() -> elevatorSubsystem.setElevatorPosition(.51), elevatorSubsystem));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
