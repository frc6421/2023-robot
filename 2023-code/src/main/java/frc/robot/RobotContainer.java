// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.WristConstants.WristAngleConstants;
import frc.robot.commands.autonomousCommands.FastFlippedTwoPieceCommand;
import frc.robot.commands.autonomousCommands.FastTwoPieceCommand;
import frc.robot.commands.autonomousCommands.FlippedThreePieceCommand;
import frc.robot.commands.autonomousCommands.FlippedTwoPieceChargeCommand;
import frc.robot.commands.autonomousCommands.FlippedTwoPieceCommand;
import frc.robot.commands.autonomousCommands.OnePieceChargeCommand;
import frc.robot.commands.autonomousCommands.ThreePieceCommand;
import frc.robot.commands.autonomousCommands.TwoPieceChargeCommand;
import frc.robot.commands.autonomousCommands.TwoPieceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;

import java.util.Map;

import javax.management.InstanceNotFoundException;

import com.fasterxml.jackson.core.base.GeneratorBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.VisionCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShuffleboardButtonManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private final ArmSubsystem armSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public final GyroSubsystem gyroSubsystem;

  private final ShuffleboardButtonManager shuffleboardButtonManager;

  // Set up controller with CommandXboxController
  private final CommandXboxController driverController;

  private final CommandXboxController testController;

  private final PowerDistribution PDP;

  private SendableChooser<Command> autoChooser;

  private OnePieceChargeCommand onePieceChargeCommand;
  private TwoPieceCommand twoPieceCommand;
  private FlippedTwoPieceCommand flippedTwoPieceCommand;
  private TwoPieceChargeCommand twoPieceChargeCommand;
  private FlippedTwoPieceChargeCommand flippedTwoPieceChargeCommand;
  private FastTwoPieceCommand fastTwoPieceCommand;
  private FastFlippedTwoPieceCommand fastFlippedTwoPieceCommand;
  private ThreePieceCommand threePieceCommand;
  private FlippedThreePieceCommand flippedThreePieceCommand;

  private static double driveNerf = 0.85;
  private static double steerNerf = 0.8;

  public static RobotStates robotState;

  // private GenericEntry armFFTest;

  // private ShuffleboardTab testTab;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    
    gyroSubsystem = new GyroSubsystem();

    driveSubsystem = new DriveSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    armSubsystem = new ArmSubsystem();
    wristSubsystem = new WristSubsystem();
    intakeSubsystem = new IntakeSubsystem();

    shuffleboardButtonManager = new ShuffleboardButtonManager();

    // testTab = Shuffleboard.getTab("Test tab");
    // armFFTest = testTab.add("arm FF test", 0).getEntry();

    
    //visionCommand = new VisionCommand(driveSubsystem);

    onePieceChargeCommand = new OnePieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, intakeSubsystem, wristSubsystem);
    twoPieceCommand = new TwoPieceCommand(driveSubsystem, elevatorSubsystem, armSubsystem, intakeSubsystem, wristSubsystem);
    flippedTwoPieceCommand = new FlippedTwoPieceCommand(driveSubsystem, elevatorSubsystem, armSubsystem, intakeSubsystem, wristSubsystem);
    twoPieceChargeCommand = new TwoPieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, intakeSubsystem, wristSubsystem);
    flippedTwoPieceChargeCommand = new FlippedTwoPieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, intakeSubsystem, wristSubsystem);
    fastTwoPieceCommand = new FastTwoPieceCommand(driveSubsystem, elevatorSubsystem, armSubsystem, intakeSubsystem, wristSubsystem);
    fastFlippedTwoPieceCommand = new FastFlippedTwoPieceCommand(driveSubsystem, elevatorSubsystem, armSubsystem, intakeSubsystem, wristSubsystem);
    threePieceCommand = new ThreePieceCommand(driveSubsystem, intakeSubsystem, wristSubsystem, armSubsystem, elevatorSubsystem);
    flippedThreePieceCommand = new FlippedThreePieceCommand(driveSubsystem, intakeSubsystem, wristSubsystem, armSubsystem, elevatorSubsystem);

  
    autoChooser = new SendableChooser<>();
    
    driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    testController = new CommandXboxController(2);

    PDP = new PowerDistribution();
    PDP.clearStickyFaults();

    

    driveSubsystem.setDefaultCommand(new RunCommand(() ->
      driveSubsystem.drive(
        MathUtil.clamp(driverController.getLeftY() * driveNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getLeftX() * driveNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getRightX() * steerNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getLeftTriggerAxis() * driveNerf, -1.0, 1.0),
        MathUtil.clamp(driverController.getRightTriggerAxis() * driveNerf, -1.0, 1.0)
        ),
        driveSubsystem));

    wristSubsystem.setDefaultCommand(new RunCommand(() ->
      wristSubsystem.setPercentPosition(testController.getLeftY() * 0.0001), wristSubsystem)); 

    armSubsystem.setDefaultCommand(new RunCommand(() ->
      armSubsystem.setPercentPosition(testController.getLeftX() * 0.0001), armSubsystem));

    elevatorSubsystem.setDefaultCommand(new RunCommand(() ->
      elevatorSubsystem.setPercentPosition(testController.getRightY() * 0.0001), elevatorSubsystem));

      
    autoChooser.setDefaultOption("1 Piece Charge", onePieceChargeCommand);
    autoChooser.addOption("Left Start Bump 2 Piece", flippedTwoPieceCommand);
    autoChooser.addOption("Right Start Bump 2 Piece", twoPieceCommand);
    autoChooser.addOption("Left Start 2 Piece Charge", flippedTwoPieceChargeCommand);
    autoChooser.addOption("Right Start 2 Piece Charge", twoPieceChargeCommand);
    autoChooser.addOption("Left Start Not Bump 2.5 Piece", fastFlippedTwoPieceCommand);
    autoChooser.addOption("Right Start Not Bump 2.5 Piece", fastTwoPieceCommand);
    autoChooser.addOption("Left Start 3 Piece", flippedThreePieceCommand);
    autoChooser.addOption("Right Start 3 Piece", threePieceCommand);
    
    Shuffleboard.getTab("Competition").add("autoChooser", autoChooser)
    .withPosition(6, 2)
    .withSize(2, 1);

    Shuffleboard.selectTab("Competition");

    robotState = RobotStates.DRIVE;

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
    driverController.start().onTrue(new RunCommand(() -> driveSubsystem.setSteerMotorsToAbsolute()));

    // Change from intake position to drive position
    // Run belts at a slow speed to hold pieces in
    driverController.y().onTrue(new InstantCommand(() -> robotState = RobotStates.DRIVE)
        .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER))));

    // Set robot to hybrid position and reverse intake
    // driverController.b().onTrue(new InstantCommand(() -> robotState = RobotStates.HYBRID)
    //     .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)))
    //     .andThen(new ParallelDeadlineGroup(new WaitCommand(0.3), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED))))
    //     .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))
    //     .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER)))));
    //driverController.b().onFalse(new InstantCommand(() -> robotState = RobotStates.DRIVE)
        //.andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)))
        //.andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER))));

    // Set robot to intake position and turn intake on for floor pickup
    driverController.a().onTrue(new InstantCommand(() -> robotState = RobotStates.INTAKE)
        .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_PICK_UP_SPEED)))
        .andThen(new WaitUntilCommand(() -> (intakeSubsystem.getIntakeVelocity() > -250)))
        .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))
        .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER))));

    // Turn intake on for substation pick up, then bring arm back in
    // driverController.x().onTrue(new ParallelDeadlineGroup(new WaitCommand(0.5), new VisionCommand(driveSubsystem))
    //     .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem)))
    //     .andThen(new WristCommand(wristSubsystem))
    //     .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_PICK_UP_SPEED)))
    //     .andThen(new WaitUntilCommand(() -> (intakeSubsystem.getIntakeVelocity() > -200)))
    //     .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))
    //     .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)))
    //     .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER))));

    // Run vision command and set arm, elevator, and wrist to correct position
     driverController.leftBumper().onTrue(
        new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem))
        .andThen(new SelectCommand(Map.ofEntries(
          Map.entry(RobotStates.LEFT_SUBSTATION, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_PICK_UP_SPEED))
            .andThen(new WaitUntilCommand(() -> (intakeSubsystem.getIntakeVelocity() > -250)))
            .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))
            .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)))
            .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER)))),
          Map.entry(RobotStates.RIGHT_SUBSTATION, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_PICK_UP_SPEED))
            .andThen(new WaitUntilCommand(() -> (intakeSubsystem.getIntakeVelocity() > -250)))
            .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))
            .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)))
            .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER)))),
          Map.entry(RobotStates.SINGLE_SUBSTATION, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_PICK_UP_SPEED))
            .andThen(new WaitUntilCommand(() -> (intakeSubsystem.getIntakeVelocity() > -250)))
            .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))
            .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)))
            .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER))))), 
          ()-> robotState)));

    // Place piece on node (hold button to spin intake, release to stop) 
    driverController.rightBumper().whileTrue(new SelectCommand(Map.ofEntries(
      Map.entry(RobotStates.DRIVE, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HYBRID_SCORE_SPEED))),
      Map.entry(RobotStates.HYBRID_LEFT, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HYBRID_SCORE_SPEED))),
      Map.entry(RobotStates.HYBRID_CENTER, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HYBRID_SCORE_SPEED))),
      Map.entry(RobotStates.HYBRID_RIGHT, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HYBRID_SCORE_SPEED))),
      Map.entry(RobotStates.HIGH_LEFT, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED))),
      Map.entry(RobotStates.HIGH_CENTER, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_CENTER_SCORE_SPEED))),
      Map.entry(RobotStates.HIGH_RIGHT, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED))),
      Map.entry(RobotStates.MID_CENTER, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_CENTER_SCORE_SPEED))),
      Map.entry(RobotStates.MID_LEFT, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED))),
      Map.entry(RobotStates.MID_RIGHT, new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SCORE_SPEED)))),
      ()-> robotState));

    driverController.rightBumper().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntakeMotors())
        .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))
        .andThen(new ParallelCommandGroup(new ArmCommand(armSubsystem), new ElevatorCommand(elevatorSubsystem), new WristCommand(wristSubsystem)))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER))));

    // driverController.povUp().onTrue(new InstantCommand(() -> WristAngleConstants.WRIST_SUBSTATION_ANGLE = WristAngleConstants.WRIST_SUBSTATION_ANGLE + 2)
    //   .andThen(new InstantCommand(() -> System.out.println(WristAngleConstants.WRIST_SUBSTATION_ANGLE))));
    // driverController.povDown().onTrue(new InstantCommand(() -> WristAngleConstants.WRIST_SUBSTATION_ANGLE = WristAngleConstants.WRIST_SUBSTATION_ANGLE - 2)
    //   .andThen(new InstantCommand(() -> System.out.println(WristAngleConstants.WRIST_SUBSTATION_ANGLE))));

    // Reset wrist encoder in case of skipping
    testController.a().onTrue(new InstantCommand(() -> wristSubsystem.resetEncoderPosition()));

    //testController.x().whileTrue(new RunCommand(() -> armSubsystem.setPercentArmPowerNoLimit(armFFTest.getDouble(0))));

    testController.y().whileTrue(new BalanceCommand(driveSubsystem));
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

  public void setIntakePowerTeleopInit()
  {
    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_HOLD_POWER);
  }

  private RobotStates getRobotSupplier()
  {
    return robotState;
  }

}
