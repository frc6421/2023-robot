// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autonomousCommands.FlippedOnePieceChargeCommand;
import frc.robot.commands.autonomousCommands.FlippedOutOfCommunityCommand;
import frc.robot.commands.autonomousCommands.FourPieceCommand;
import frc.robot.commands.autonomousCommands.OnePieceChargeCommand;
import frc.robot.commands.autonomousCommands.OutOfCommunityCommand;
import frc.robot.commands.autonomousCommands.PathPlannerOnePieceChargeCommand;
import frc.robot.commands.autonomousCommands.TestAutoCommand;
import frc.robot.commands.SubstationVisionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.ArmElevatorCommand;
import frc.robot.commands.SubstationGamePieceVisionCommand;
import frc.robot.commands.ArmElevatorCommand.PlaceStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  private final LimelightSubsystem limelightSubsystem;

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

  private final SubstationVisionCommand substationVisionCommand;
  private final SubstationGamePieceVisionCommand substationGamePieceVisionCommand;

  private SendableChooser<Command> autoChooser;

  private TestAutoCommand testAutoCommand;
  private FourPieceCommand fourPieceAutoCommand;

  //private OnePieceChargeCommand onePieceChargeCommand;
  //private FlippedOnePieceChargeCommand flippedOnePieceChargeCommand;

  private OutOfCommunityCommand outOfCommunityCommand;
  private FlippedOutOfCommunityCommand flippedOutOfCommunityCommand;

  private PathPlannerOnePieceChargeCommand pathPlannerOnePieceChargeCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    
    gyroSubsystem = new GyroSubsystem();
    driveSubsystem = new DriveSubsystem();
    limelightSubsystem = new LimelightSubsystem();

    elevatorSubsystem = new ElevatorSubsystem();
    armSubsystem = new ArmSubsystem();
    grabberSubsystem = new GrabberSubsystem();
    intakeSubsystem = new IntakeSubsystem();

    testAutoCommand = new TestAutoCommand(driveSubsystem);
    fourPieceAutoCommand = new FourPieceCommand(driveSubsystem);

    //onePieceChargeCommand = new OnePieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);
    //flippedOnePieceChargeCommand = new FlippedOnePieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);
    outOfCommunityCommand = new OutOfCommunityCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);
    flippedOutOfCommunityCommand = new FlippedOutOfCommunityCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);

    pathPlannerOnePieceChargeCommand = new PathPlannerOnePieceChargeCommand(driveSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem);

    autoChooser = new SendableChooser<>();
    

    substationVisionCommand = new SubstationVisionCommand(driveSubsystem);
    substationGamePieceVisionCommand = new SubstationGamePieceVisionCommand(driveSubsystem);

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
        driverController.getRightX() * .4, 
        driverController.getLeftTriggerAxis() * DriveConstants.DRIVE_NERF_JOYSTICK_MULTIPLIER,
        driverController.getRightTriggerAxis() * DriveConstants.DRIVE_NERF_JOYSTICK_MULTIPLIER), driveSubsystem));
      
    // elevatorSubsystem.setDefaultCommand(new RunCommand(() -> 
    // elevatorSubsystem.goToPosition(-driverController.getRightY()), elevatorSubsystem)
    // );
    
      armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.setPercentPosition(copilotController.getRightY()), armSubsystem));

      elevatorSubsystem.setDefaultCommand(new RunCommand(() -> elevatorSubsystem.setPercentPosition(copilotController.getLeftY()), elevatorSubsystem));
    
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
   
    autoChooser.setDefaultOption("TestAuto", testAutoCommand);
    autoChooser.addOption("Right Start 4 Piece", fourPieceAutoCommand);
    //autoChooser.addOption("Right Start 1 Piece Charge", onePieceChargeCommand);
    //autoChooser.addOption("Left Start 1 Piece Charge", flippedOnePieceChargeCommand);
    autoChooser.addOption("Right Start Out of Community", outOfCommunityCommand);
    autoChooser.addOption("Left Start out of community", flippedOutOfCommunityCommand);
    SmartDashboard.putData("autoChooser", autoChooser);

    PDP = new PowerDistribution();
    PDP.clearStickyFaults();

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

    driverController.b().whileTrue(substationVisionCommand);

    //driverController.x().whileTrue(substationGamePieceVisionCommand);

    driverController.rightBumper().whileTrue(new InstantCommand(() -> grabberSubsystem.toggleGrabber()));

    // driverController.y().onTrue(
    //   new InstantCommand(() -> armSubsystem.setSetPoint(armSubsystem.getArmDegreePosition() + 5))
    //   .andThen(new RunCommand(() -> armSubsystem.setPosition(armSubsystem.getSetPoint()), armSubsystem))
    //   .andThen(new InstantCommand(() -> grabberSubsystem.toggleGrabber(), grabberSubsystem)));

    copilotController.povLeft().onTrue(new InstantCommand(() -> BlinkinSubsystem.blinkinYellowSet()));
    copilotController.povRight().onTrue(new InstantCommand(() -> BlinkinSubsystem.blinkinVioletSet()));

    copilotController.rightBumper().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.UP));
    copilotController.leftBumper().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.HYBRID));
    copilotController.y().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.HIGH));
    copilotController.b().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.MID));
    copilotController.x().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.SUBSTATION));
    copilotController.a().onTrue(new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.FLOOR));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoConstants.eventMap.put("armHigh", new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.HIGH));
    AutoConstants.eventMap.put("openGrabber", new InstantCommand(() -> grabberSubsystem.toggleGrabber()));
    AutoConstants.eventMap.put("armIn", new ArmElevatorCommand(elevatorSubsystem, armSubsystem, PlaceStates.HYBRID));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    driveSubsystem::getPose2d, // Pose2d supplier
    driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    driveSubsystem.swerveKinematics, // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
    AutoConstants.eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );

    // An example command will be run in autonomous
    //return autoChooser.getSelected();
    return pathPlannerOnePieceChargeCommand;
  }
}
