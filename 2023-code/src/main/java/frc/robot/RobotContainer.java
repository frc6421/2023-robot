// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.IntakeConstants;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmDownThenReleaseCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.ScoringVisionCommand;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.SubstationGamePieceVisionCommand;
import frc.robot.commands.ArmElevatorCommand.PlaceStates;
import frc.robot.commands.IntakeArmCommand.IntakePlaceStates;
import frc.robot.subsystems.BlinkinSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  //private final LimelightSubsystem limelightSubsystem;

  private final ElevatorSubsystem elevatorSubsystem;

  private final GrabberSubsystem grabberSubsystem;

  private final IntakeSubsystem intakeSubsystem;

  // Set up controller with CommandXboxController
  private final CommandXboxController driverController;
  
  private final CommandXboxController copilotController;

  private final ArmSubsystem armSubsystem;
  public GyroSubsystem gyroSubsystem;
  private final PowerDistribution PDP;
  //private final ArmElevatorCommand armElevatorCommand;

  SendableChooser<PlaceStates> armElevatorPos;

  public static Compressor compressor;

  private final PneumaticHub pneumaticHub;

  private final SubstationVisionCommand substationVisionCommand;
  private final SubstationGamePieceVisionCommand substationGamePieceVisionCommand;
  private final ScoringVisionCommand scoringVisionCommand;

  private SendableChooser<Command> autoChooser;

  private FourPieceCommand fourPieceAutoCommand;
  private OnePieceChargeCommand onePieceChargeCommand;
  private TwoPieceCommand twoPieceCommand;
  private FlippedTwoPieceCommand flippedTwoPieceCommand;
  private TwoPieceChargeCommand twoPieceChargeCommand;
  private FlippedTwoPieceChargeCommand flippedTwoPieceChargeCommand;

  private static double driveNerf = 0.85;
  private static double steerNerf = 0.8; //0.5

  public static boolean isLeftCone;

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
    twoPieceCommand = new TwoPieceCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem, intakeSubsystem);
    flippedTwoPieceCommand = new FlippedTwoPieceCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem, intakeSubsystem);
    twoPieceChargeCommand = new TwoPieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);
    flippedTwoPieceChargeCommand = new FlippedTwoPieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);

    autoChooser = new SendableChooser<>();
    

    substationVisionCommand = new SubstationVisionCommand(driveSubsystem);
    substationGamePieceVisionCommand = new SubstationGamePieceVisionCommand(driveSubsystem);
    scoringVisionCommand = new ScoringVisionCommand(driveSubsystem);

    driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    PDP = new PowerDistribution();
    PDP.clearStickyFaults();

    copilotController = new CommandXboxController(OperatorConstants.COPILOT_CONTROLLER_PORT);

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
    
      //armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.setPercentPosition(copilotController.getRightY()), armSubsystem));

      // elevatorSubsystem.setDefaultCommand(new RunCommand(() -> elevatorSubsystem.setPercentPosition(copilotController.getLeftY()), elevatorSubsystem));
    
      //intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.setPercentPosition(copilotController.getLeftY()), intakeSubsystem));
      //intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.setIntakeSpeed(copilotController.getRightY()), intakeSubsystem));

      armElevatorPos = new SendableChooser<>();
      armElevatorPos.setDefaultOption("Mid", PlaceStates.MID);
      armElevatorPos.addOption("Intake", PlaceStates.INTAKE);
      armElevatorPos.addOption("Floor", PlaceStates.FLOOR);
      armElevatorPos.addOption("High", PlaceStates.HIGH);
      armElevatorPos.addOption("Substation", PlaceStates.SUBSTATION);
    
    //autoChooser.addOption("Right Start 4 Piece", fourPieceAutoCommand);
    autoChooser.setDefaultOption("1 Piece Charge", onePieceChargeCommand);
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
    SmartDashboard.putNumber("Air pressure", compressor.getPressure());

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

    driverController.leftBumper().whileTrue(scoringVisionCommand);

    //driverController.x().whileTrue(substationGamePieceVisionCommand);

    driverController.rightBumper().whileTrue(new InstantCommand(() -> grabberSubsystem.toggleGrabber()));

    // Arm up and intake turned low speed then intake in
    driverController.y().onTrue(new ParallelCommandGroup(new InstantCommand(() -> driveNerf = 0.75))
        .andThen(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE))
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0.2)))
        .andThen(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.DRIVE))
        .andThen(new WaitCommand(.5))
        .andThen(new InstantCommand(() -> intakeSubsystem.setEncoderPosition(IntakeConstants.INTAKE_DRIVE_ANGLE))));
    // Reverse intake for hybrid
    driverController.b().onTrue(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.HYBRID)
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(-0.5), intakeSubsystem)));

    // driverController.a().onTrue(new InstantCommand(() -> driveNerf = 0.25)
    // .andThen(new IntakeDownCommand(intakeSubsystem))
    // .andThen(new InstantCommand(() -> driveNerf = 0.75)));
    // Intake floor pickup
    driverController.a().onTrue(new ParallelCommandGroup(new InstantCommand(() -> driveNerf = 0.25))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(1), intakeSubsystem))
        .andThen(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.FLOOR))
        .andThen(new WaitCommand(.5))
        .andThen(new InstantCommand(() -> intakeSubsystem.setEncoderPosition(IntakeConstants.INTAKE_FLOOR_ANGLE)))
        .andThen(new WaitUntilCommand(() -> intakeSubsystem.getIntakeVelocity() < 500))
        .andThen(new ParallelCommandGroup(new InstantCommand(() -> driveNerf = 0.75)))
        .andThen(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE))
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0.2)))
        .andThen(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.DRIVE))
        .andThen(new WaitCommand(.5))
        .andThen(new InstantCommand(() -> intakeSubsystem.setEncoderPosition(IntakeConstants.INTAKE_DRIVE_ANGLE))));

      driverController.x().onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(1), intakeSubsystem)
        .andThen(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.SINGLE))
        .andThen(new InstantCommand(() -> intakeSubsystem.setPosition(IntakeConstants.INTAKE_SINGLE_ANGLE)))
        .andThen(new WaitUntilCommand(() -> intakeSubsystem.getIntakeVelocity() < 500))
        .andThen(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE))
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0.2)))
        .andThen(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.DRIVE)));


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
    driverController.povDown().whileTrue(new ArmDownThenReleaseCommand(armSubsystem, grabberSubsystem));
    //copilotController.start().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeSpeed(copilotController.getRightY()), intakeSubsystem));

    // Set LEDs to yellow
    copilotController.povUp().onTrue(new InstantCommand(() -> BlinkinSubsystem.blinkinYellowSet()));

    // Set LEDs to purple
    copilotController.povDown().onTrue(new InstantCommand(() -> BlinkinSubsystem.blinkinVioletSet()));

    copilotController.leftTrigger(0.2).onTrue(new InstantCommand(() -> isLeftCone = true));
    copilotController.rightTrigger(0.2).onTrue(new InstantCommand(() -> isLeftCone = false));

    // Up button (arm drive position)
    copilotController.rightBumper().onTrue(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE)
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.DRIVE)));
    
    // High button
    copilotController.y().onTrue(new ParallelDeadlineGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.HIGH),
      new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.HIGH)));

    // Mid button
    copilotController.b().onTrue(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.MID)
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.MID)));

    // Substation arm/elevator button
    copilotController.x().onTrue(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.SUBSTATION)
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.SUBSTATION)));

    // Transfer button
    copilotController.a().onTrue(new InstantCommand(() -> grabberSubsystem.release())
        .andThen(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0.2)))
        .andThen(new IntakeArmCommand(intakeSubsystem, IntakePlaceStates.DRIVE))
        .andThen(new ElevatorCommand(elevatorSubsystem, ElevatorCommand.PlaceStates.TRANSFER))
        .andThen(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.TRANSFER))
        .andThen(new ParallelDeadlineGroup(new WaitCommand(0.15), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(BlinkinSubsystem.isPurpled() ? 0.2 : -0.2))))
        .andThen(new ParallelDeadlineGroup(new WaitCommand(0.6), /*new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0)),*/ new InstantCommand(() -> grabberSubsystem.grab())))
        .andThen(new ParallelDeadlineGroup(new ArmCommand(armSubsystem, ArmCommand.PlaceStates.DRIVE), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0)))));
    
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
