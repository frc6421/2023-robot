// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autonomousCommands.FlippedTwoPieceChargeCommand;
import frc.robot.commands.autonomousCommands.FlippedTwoPieceCommand;
import frc.robot.commands.autonomousCommands.FourPieceCommand;
import frc.robot.commands.autonomousCommands.OnePieceChargeCommand;
import frc.robot.commands.autonomousCommands.TwoPieceChargeCommand;
import frc.robot.commands.autonomousCommands.TwoPieceCommand;
import frc.robot.commands.SubstationVisionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.time.Instant;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmDownThenReleaseCommand;
import frc.robot.commands.ArmElevatorCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.SubstationGamePieceVisionCommand;
import frc.robot.commands.ArmElevatorCommand.PlaceStates;
import frc.robot.commands.IntakeArmCommand.IntakePlaceStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

import edu.wpi.first.networktables.GenericEntry;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  //private final LimelightSubsystem limelightSubsystem;

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
  private ShuffleboardTab intakeTab;
  private GenericEntry intakeSetFF;
  public GyroSubsystem gyroSubsystem;
  private final PowerDistribution PDP;
  //private final ArmElevatorCommand armElevatorCommand;

  SendableChooser<PlaceStates> armElevatorPos;

  private static Compressor compressor;

  private final PneumaticHub pneumaticHub;

  private final SubstationVisionCommand substationVisionCommand;
  private final SubstationGamePieceVisionCommand substationGamePieceVisionCommand;

  private SendableChooser<Command> autoChooser;

  private FourPieceCommand fourPieceAutoCommand;
  private OnePieceChargeCommand onePieceChargeCommand;
  private TwoPieceCommand twoPieceCommand;
  private FlippedTwoPieceCommand flippedTwoPieceCommand;
  private TwoPieceChargeCommand twoPieceChargeCommand;
  private FlippedTwoPieceChargeCommand flippedTwoPieceChargeCommand;

  //Creates a double that takes the desired drive voltage and divides it by the current voltage
  private double voltageRatio;

  private static double driveNerf = 0.75;
  private static double steerNerf = 0.5;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    
    gyroSubsystem = new GyroSubsystem();
    driveSubsystem = new DriveSubsystem();
    //limelightSubsystem = new LimelightSubsystem();

    elevatorSubsystem = new ElevatorSubsystem();
    armSubsystem = new ArmSubsystem();
    grabberSubsystem = new GrabberSubsystem();
    intakeSubsystem = new IntakeSubsystem();

    fourPieceAutoCommand = new FourPieceCommand(driveSubsystem);
    onePieceChargeCommand = new OnePieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);
    twoPieceCommand = new TwoPieceCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);
    flippedTwoPieceCommand = new FlippedTwoPieceCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);
    twoPieceChargeCommand = new TwoPieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);
    flippedTwoPieceChargeCommand = new FlippedTwoPieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);

    autoChooser = new SendableChooser<>();
    

    substationVisionCommand = new SubstationVisionCommand(driveSubsystem);
    substationGamePieceVisionCommand = new SubstationGamePieceVisionCommand(driveSubsystem);

    driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    PDP = new PowerDistribution();
    PDP.clearStickyFaults();

    //used to make modify controller inputs TODO see if this is what we want + test + pass in or modify here?
    voltageRatio = DriveConstants.DRIVE_VOLTAGE / PDP.getVoltage();
    copilotController = new CommandXboxController(OperatorConstants.COPILOT_CONTROLLER_PORT);


    SmartDashboard.putNumber("LeftY", driverController.getLeftY());
    SmartDashboard.putNumber("LeftX", driverController.getLeftX());
    SmartDashboard.putNumber("RightX", driverController.getRightX());

    SmartDashboard.putNumber("Arm Degree Position: ", armSubsystem.getArmDegreePosition());
    SmartDashboard.putNumber("Arm Feed Forward: ", armSubsystem.getFeedForward());

    driveSubsystem.setDefaultCommand(new RunCommand(() ->
      driveSubsystem.drive(
        MathUtil.clamp(driverController.getLeftY() * driveNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getLeftX() * driveNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getRightX() * steerNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getLeftTriggerAxis() * driveNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getRightTriggerAxis() * driveNerf, 1.0, 1.0)
        ),
        driveSubsystem));
      
    // elevatorSubsystem.setDefaultCommand(new RunCommand(() -> 
    // elevatorSubsystem.goToPosition(-driverController.getRightY()), elevatorSubsystem)
    // );
    
      armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.setPercentPosition(copilotController.getRightY()), armSubsystem));

      // elevatorSubsystem.setDefaultCommand(new RunCommand(() -> elevatorSubsystem.setPercentPosition(copilotController.getLeftY()), elevatorSubsystem));
    
      intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.setPercentPosition(copilotController.getLeftY()), intakeSubsystem));
      //intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.setIntakeSpeed(copilotController.getRightY()), intakeSubsystem));
      
      intakeTab = Shuffleboard.getTab("Intake Tab");

      intakeSetFF = intakeTab.add("Set Intake Arm FF: ", 0)
        .getEntry();
      
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
   
    //autoChooser.addOption("Right Start 4 Piece", fourPieceAutoCommand);
    autoChooser.addOption("1 Piece Charge", onePieceChargeCommand);
    autoChooser.addOption("Left Start 2 Piece", flippedTwoPieceCommand);
    autoChooser.addOption("Right Start 2 Piece", twoPieceCommand);
    autoChooser.addOption("Left Start 2 Piece Charge", flippedTwoPieceChargeCommand);
    autoChooser.addOption("Right Start 2 Piece Charge", twoPieceChargeCommand);
    SmartDashboard.putData("autoChooser", autoChooser);

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

    driverController.leftBumper().whileTrue(substationVisionCommand);

    //driverController.x().whileTrue(substationGamePieceVisionCommand);

    driverController.rightBumper().whileTrue(new InstantCommand(() -> grabberSubsystem.toggleGrabber()));

    // Arm up and intake turned low speed then intake in
    driverController.y().onTrue(new ParallelCommandGroup(new InstantCommand(() -> driveNerf = 0.75), new InstantCommand(() -> steerNerf = 0.5))
        .andThen(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE))
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0.5)))
        .andThen(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.DRIVE)));
    // Reverse intake for hybrid
    driverController.b().onTrue(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.HYBRID)
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(-0.5), intakeSubsystem)));
    // Intake floor pickup
    driverController.a().onTrue(new ParallelCommandGroup(new InstantCommand(() -> driveNerf = 0.5), new InstantCommand(() -> steerNerf = 0.25))
        .andThen(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.FLOOR))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(1), intakeSubsystem)));
        // .andThen(new InstantCommand(() -> driveSubsystem.drive(MathUtil.clamp(driverController.getLeftY() * DriveConstants.DRIVE_NERF_JOYSTICK_MULTIPLIER, -1.0, 1.0),
        //   MathUtil.clamp(driverController.getLeftX() * 0.5, -1.0, 1.0),
        //   MathUtil.clamp(driverController.getRightX() * 0.25, -1.0, 1.0),
        //   MathUtil.clamp(driverController.getLeftTriggerAxis() * 0.5, -1.0, 1.0),
        //   MathUtil.clamp(driverController.getRightTriggerAxis() * 0.5, 1.0, 1.0)), driveSubsystem)));

    //driverController.a().onTrue(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.SWAP));
    //driverController.b().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeArmAngleWithGrav(intakeSetFF.getDouble(0)), intakeSubsystem));

    // driverController.y().onTrue(
    //   new InstantCommand(() -> armSubsystem.setSetPoint(armSubsystem.getArmDegreePosition() + 5))
    //   .andThen(new RunCommand(() -> armSubsystem.setPosition(armSubsystem.getSetPoint()), armSubsystem))
    //   .andThen(new InstantCommand(() -> grabberSubsystem.toggleGrabber(), grabberSubsystem)));
   // driverController.x().whileTrue(new ArmDownThenReleaseCommand(armSubsystem, grabberSubsystem));
    driverController.x().whileTrue(new ArmDownThenReleaseCommand(armSubsystem, grabberSubsystem));
    //copilotController.start().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeSpeed(copilotController.getRightY()), intakeSubsystem));
    copilotController.povLeft().onTrue(new InstantCommand(() -> BlinkinSubsystem.blinkinYellowSet()));
    copilotController.povRight().onTrue(new InstantCommand(() -> BlinkinSubsystem.blinkinVioletSet()));

    copilotController.rightBumper().onTrue(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE)
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE)));
    //copilotController.leftBumper().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.HYBRID));
    copilotController.y().onTrue(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.HIGH)
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.HIGH)));
    copilotController.b().onTrue(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.MID)
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.MID)));
    copilotController.x().onTrue(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.SUBSTATION)
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.SUBSTATION)));
    copilotController.a().onTrue(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.FLOOR)
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.FLOOR)));

        //EXPERIMENTAL UNTIL TESTED\\
    copilotController.leftBumper().onTrue(new InstantCommand(() -> grabberSubsystem.release())
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0)))
        .andThen(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.DRIVE))
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.TRANSFER))
        .andThen(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.TRANSFER))
        .andThen(new ParallelDeadlineGroup(new WaitCommand(0.5), new InstantCommand(() -> grabberSubsystem.grab())))
        .andThen(new ParallelDeadlineGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE), new ParallelDeadlineGroup(new WaitCommand(0.5), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(-0.1))))));
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
