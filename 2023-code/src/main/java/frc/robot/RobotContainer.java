// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autonomousCommands.FlippedTwoPieceChargeCommand;
import frc.robot.commands.autonomousCommands.FlippedTwoPieceCommand;
import frc.robot.commands.autonomousCommands.OnePieceChargeCommand;
import frc.robot.commands.autonomousCommands.TwoPieceChargeCommand;
import frc.robot.commands.autonomousCommands.TwoPieceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  //private final LimelightSubsystem limelightSubsystem;

  private final ElevatorSubsystem elevatorSubsystem;

  private final WristSubsystem wristSubsystem;

  // Set up controller with CommandXboxController
  private final CommandXboxController driverController;
  
  private final CommandXboxController copilotController;

  private final CommandXboxController testController;

  NetworkTableEntry wristP;
  NetworkTableEntry wristAngle;
  NetworkTableEntry armAngle;

  private final ArmSubsystem armSubsystem;
  public GyroSubsystem gyroSubsystem;
  private final PowerDistribution PDP;

  private final IntakeSubsystem intakeSubsystem;

  private SendableChooser<Command> autoChooser;

  private OnePieceChargeCommand onePieceChargeCommand;
  private TwoPieceCommand twoPieceCommand;
  private FlippedTwoPieceCommand flippedTwoPieceCommand;
  private TwoPieceChargeCommand twoPieceChargeCommand;
  private FlippedTwoPieceChargeCommand flippedTwoPieceChargeCommand;

  private static double driveNerf = 0.85;
  private static double steerNerf = 0.8; //0.5

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    
    gyroSubsystem = new GyroSubsystem();
    driveSubsystem = new DriveSubsystem();

    elevatorSubsystem = new ElevatorSubsystem();
    armSubsystem = new ArmSubsystem();
    wristSubsystem = new WristSubsystem();
    intakeSubsystem = new IntakeSubsystem();

    onePieceChargeCommand = new OnePieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem);
    twoPieceCommand = new TwoPieceCommand(driveSubsystem, elevatorSubsystem, armSubsystem);
    flippedTwoPieceCommand = new FlippedTwoPieceCommand(driveSubsystem, elevatorSubsystem, armSubsystem);
    twoPieceChargeCommand = new TwoPieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem);
    flippedTwoPieceChargeCommand = new FlippedTwoPieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem);

    

    autoChooser = new SendableChooser<>();
    
    driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    PDP = new PowerDistribution();
    PDP.clearStickyFaults();

    copilotController = new CommandXboxController(OperatorConstants.COPILOT_CONTROLLER_PORT);

    testController = new CommandXboxController(2);

    driveSubsystem.setDefaultCommand(new RunCommand(() ->
      driveSubsystem.drive(
        MathUtil.clamp(driverController.getLeftY() * driveNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getLeftX() * driveNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getRightX() * steerNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getLeftTriggerAxis() * driveNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getRightTriggerAxis() * driveNerf, 1.0, 1.0)
        ),
        driveSubsystem));

    wristSubsystem.setDefaultCommand(new RunCommand(() ->
      wristSubsystem.setPercentPosition(testController.getLeftX()), wristSubsystem)); 

    armSubsystem.setDefaultCommand(new RunCommand(() ->
      armSubsystem.setPercentPosition(testController.getLeftY()), armSubsystem));

    elevatorSubsystem.setDefaultCommand(new RunCommand(() ->
      elevatorSubsystem.setPercentPosition(testController.getRightY()), elevatorSubsystem));

      
    autoChooser.setDefaultOption("1 Piece Charge", onePieceChargeCommand);
    autoChooser.addOption("Left Start 2 Piece", flippedTwoPieceCommand);
    autoChooser.addOption("Right Start 2 Piece", twoPieceCommand);
    autoChooser.addOption("Left Start 2 Piece Charge", flippedTwoPieceChargeCommand);
    autoChooser.addOption("Right Start 2 Piece Charge", twoPieceChargeCommand);
    SmartDashboard.putData("autoChooser", autoChooser);

      SmartDashboard.putNumber("wrist p", 0);
     wristP = SmartDashboard.getEntry("wrist p");

     SmartDashboard.putNumber("wrist angle", 136.5);
      wristAngle = SmartDashboard.getEntry("wrist angle");

    SmartDashboard.putNumber("arm angle", -28);
    armAngle = SmartDashboard.getEntry("arm angle");

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

    driverController.back().onTrue(new InstantCommand(() -> GyroSubsystem.zeroGyro())); 
    driverController.start().whileTrue(new RunCommand(() -> driveSubsystem.setSteerMotorsToAbsolute()));

    // Arm up and intake turned low speed then intake in
    driverController.y().onTrue(new ParallelCommandGroup(new InstantCommand(() -> driveNerf = 0.75))
        .andThen(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE))
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE))
        .andThen(new WaitCommand(.5))
        );
    // Reverse intake for hybrid
    

    // driverController.a().onTrue(new InstantCommand(() -> driveNerf = 0.25)
    // .andThen(new IntakeDownCommand(intakeSubsystem))
    // .andThen(new InstantCommand(() -> driveNerf = 0.75)));
    // Intake floor pickup
    driverController.a().onTrue(new ParallelCommandGroup(new InstantCommand(() -> driveNerf = 0.25))
        .andThen(new WaitCommand(.5))
        .andThen(new ParallelCommandGroup(new InstantCommand(() -> driveNerf = 0.75)))
        .andThen(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE))
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE))
        .andThen(new WaitCommand(.5)));

    // Set LEDs to yellow
    copilotController.povUp().onTrue(new InstantCommand(() -> BlinkinSubsystem.blinkinYellowSet()));

    // Set LEDs to purple
    copilotController.povDown().onTrue(new InstantCommand(() -> BlinkinSubsystem.blinkinVioletSet()));

    // Up button (arm drive position)
    copilotController.rightBumper().onTrue(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE)
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE)));
    
    // High button
    copilotController.y().onTrue(new ParallelDeadlineGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.HIGH),
      new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.HIGH)));

    // Mid button
    copilotController.b().onTrue(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.MID)
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.MID)));

    
    testController.y().onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(-1)));

    testController.a().onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0)));

    testController.b().onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(1)));

    testController.x().onTrue(new RunCommand(() -> wristSubsystem.setWristP(wristP.getDouble(0))));

    testController.rightBumper().onTrue(new InstantCommand(() -> wristSubsystem.setWristAngleWithGrav(wristAngle.getDouble(136.5))));

    testController.leftBumper().onTrue(new InstantCommand(() -> armSubsystem.setArmAngleWithGrav(armAngle.getDouble(-28))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Returns the selected autonomous command from the sendable chooser
    return autoChooser.getSelected();
  }
}
