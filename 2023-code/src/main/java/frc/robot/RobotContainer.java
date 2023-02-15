// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmElevatorCommand;
import frc.robot.commands.ArmElevatorCommand.PlaceStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  private final ElevatorSubsystem elevatorSubsystem;

  private final GrabberSubsystem grabberSubsystem;

  private final IntakeSubsystem intakeSubsystem;

  // Set up controller with CommandXboxController
  private final CommandXboxController driverController;

  private ShuffleboardTab elevatorTab;
  private GenericEntry elevatorFFTestingEntry;
  private GenericEntry elevatorPositionTestEntry;
  private GenericEntry elevatorPTestingEntry;
  
  private final CommandXboxController copilotController;

  private final ArmSubsystem armSubsystem;

  private GenericEntry armSetFFTestEntry;
  private GenericEntry armSetPosTestEntry;
  private GenericEntry armSetPowerTestEntry;
  private GenericEntry armSetPTestEntry;

  private ShuffleboardTab armTab;
  private ShuffleboardTab intakeGrabberTab;
  public GyroSubsystem gyroSubsystem;
  private final PowerDistribution PDP;
  //private final ArmElevatorCommand armElevatorCommand;

  SendableChooser<PlaceStates> armElevatorPos;

  private static Compressor compressor;

  private final PneumaticHub pneumaticHub;

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    gyroSubsystem = new GyroSubsystem();
    
    driveSubsystem = new DriveSubsystem();

    elevatorSubsystem = new ElevatorSubsystem();
    armSubsystem = new ArmSubsystem();

    grabberSubsystem = new GrabberSubsystem();
    intakeSubsystem = new IntakeSubsystem();


    driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    copilotController = new CommandXboxController(OperatorConstants.COPILOT_CONTROLLER_PORT);

    



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
      
    // elevatorSubsystem.setDefaultCommand(new RunCommand(() -> 
    // elevatorSubsystem.goToPosition(-driverController.getRightY()), elevatorSubsystem)
    // );
    
      armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.setPercentPosition(copilotController.getLeftY()), armSubsystem));

      elevatorSubsystem.setDefaultCommand(new RunCommand(() -> elevatorSubsystem.setPercentPosition(copilotController.getRightY()), elevatorSubsystem));
    
      armTab = Shuffleboard.getTab("Arm Tab");
        
      armSetFFTestEntry = armTab.add("Set Arm FF: ", 0) 
              .getEntry();
    
      armSetPosTestEntry = armTab.add("Set Arm Degree Position: ", 0) 
              .getEntry();

      armSetPowerTestEntry = armTab.add("Set Arm Power: ", 0) 
              .getEntry();

      armSetPTestEntry = armTab.add("Set Arm P Value: ", 0) 
              .getEntry();

      armElevatorPos = new SendableChooser<>();
      armElevatorPos.setDefaultOption("Mid", PlaceStates.MID);
      armElevatorPos.addOption("Intake", PlaceStates.INTAKE);
      armElevatorPos.addOption("Floor", PlaceStates.FLOOR);
      armElevatorPos.addOption("High", PlaceStates.HIGH);
      armElevatorPos.addOption("Substation", PlaceStates.SUBSTATION);
      SmartDashboard.putData("Arm elevator position", armElevatorPos);
    
      //TODO: Testing purposes only
      //copilotController.x().whileTrue(new RunCommand(()-> armSubsystem.setArmAngleWithGrav(armSetPosTestEntry.getDouble(0))));
      //copilotController.a().whileTrue(new RunCommand(()-> armSubsystem.setPercentArmPowerNoLimit(armSetPowerTestEntry.getDouble(0)), armSubsystem));
      //copilotController.y().whileTrue(new RunCommand(()-> armSubsystem.setArmP(armSetPTestEntry.getDouble(0)), armSubsystem));

    copilotController.x().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.MID));
    copilotController.y().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.UP));
    copilotController.a().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.FLOOR));
    copilotController.b().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.HIGH));


    elevatorTab = Shuffleboard.getTab("Elevator Tab");

    elevatorFFTestingEntry = elevatorTab.add("Set Elevator FF: ", 0)
      .getEntry();

    elevatorPTestingEntry = elevatorTab.add("Set Elevator P: ", 0)
      .getEntry();

    elevatorPositionTestEntry = elevatorTab.add("Set Elevator Pos: ", 0)
      .getEntry();
    
    //driverController.x().whileTrue(new RunCommand(() -> elevatorSubsystem.goToPosition(elevatorFFTestingEntry.getDouble(0)), elevatorSubsystem));
    
    //driverController.y().whileTrue(new RunCommand(() -> elevatorSubsystem.setP(elevatorPTestingEntry.getDouble(0)), elevatorSubsystem));
    
    //driverController.b().whileTrue(new RunCommand(() -> elevatorSubsystem.setElevatorPosition(elevatorPositionTestEntry.getDouble(0)), elevatorSubsystem));
   
    PDP = new PowerDistribution();
    PDP.clearStickyFaults();

    driverController.rightBumper().whileTrue(new InstantCommand(() -> grabberSubsystem.toggleGrabber()));

    //configures the Pneumatic Hub
    pneumaticHub = new PneumaticHub();
    pneumaticHub.clearStickyFaults();

    //configures the compressor
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    compressor.enableAnalog(95, 120);

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
    driverController.back().onTrue(new InstantCommand(() -> GyroSubsystem.zeroGyro())); 
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
