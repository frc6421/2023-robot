// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  // Set up controller with CommandXboxController
  private final CommandXboxController driverController;

  private final ArmSubsystem armSubsystem;

  private GenericEntry armSetFFTestEntry;
  private GenericEntry armSetPosTestEntry;

  private ShuffleboardTab armTab;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();

    armSubsystem = new ArmSubsystem();

    driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    



    SmartDashboard.putNumber("LeftY", driverController.getLeftY());
    SmartDashboard.putNumber("LeftX", driverController.getLeftX());
    SmartDashboard.putNumber("RightX", driverController.getRightX());

    SmartDashboard.putNumber("Arm Degree Position: ", armSubsystem.getArmDegreePosition());
    SmartDashboard.putNumber("Arm Feed Forward: ", armSubsystem.getFeedForward());

    driveSubsystem.setDefaultCommand(new RunCommand(() ->
      driveSubsystem.drive(
        driverController.getLeftY() * DriveConstants.DRIVE_NERF_JOYSTICK_MULTIPLIER,
        driverController.getLeftX() * DriveConstants.DRIVE_NERF_JOYSTICK_MULTIPLIER,
        driverController.getRightX() * .75, 
        driverController.getLeftTriggerAxis() * DriveConstants.DRIVE_NERF_JOYSTICK_MULTIPLIER,
        driverController.getRightTriggerAxis() * DriveConstants.DRIVE_NERF_JOYSTICK_MULTIPLIER), driveSubsystem));
    
      armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.setPercentArmPower(driverController.getRightY()), armSubsystem));
    
      armTab = Shuffleboard.getTab("Arm Tab");
        
      armSetFFTestEntry = armTab.add("Set Arm FF: ", 0) 
              .getEntry();
    
      armSetPosTestEntry = armTab.add("Set Arm Degree Position: ", 0) 
              .getEntry();
    
      //TODO: Testing purposes only
      armSubsystem.setDefaultCommand(new RunCommand(()-> armSubsystem.setPercentArmPower(driverController.getRightY()), armSubsystem));
      driverController.x().whileTrue(new RunCommand(()-> armSubsystem.setArmAngleAndFF(armSetPosTestEntry.getDouble(0), armSetFFTestEntry.getDouble(0)), armSubsystem));
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
//TODO turn to angle buttons
    driverController.y().onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro())); 
    driverController.start().whileTrue(new RunCommand(() -> driveSubsystem.setSteerMotorsToAbsolute()));
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
