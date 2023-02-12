// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private final DoubleSolenoid intakePiston;

  //TODO Maybe an encoder here later

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    //Constructs and sets up intake motor controllers and solenoids
    leftMotor = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.INTAKE_PISTON_FORWARD_CHANNEL, IntakeConstants.INTAKE_PISTON_REVERSE_CHANNEL);

    leftMotor.restoreFactoryDefaults();
    leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    leftMotor.setInverted(false);
    leftMotor.setSmartCurrentLimit(60);
    
    rightMotor.restoreFactoryDefaults();
    rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightMotor.setInverted(true);
    rightMotor.setSmartCurrentLimit(60); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Brings the intake pistons inside the robot
   */
  public void intakeIn() {
    intakePiston.set(Value.kForward);
  }

  /**
   * Puts the intake pistons outside the robot
   */
  public void intakeOut() {
    intakePiston.set(Value.kReverse);
  }

  /**
   * Sets the speed of the intake motors by percentage
   * @param speed double value -1.0 to 1.0
   */
  public void setIntakeSpeed(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    leftMotor.set(speed);
    rightMotor.set(speed);   //TODO set a motor to follow or have different methods for right and left?
  }

  /**
   * Stops the intake motors
   */
  public void stopIntakeMotors(){
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }


  /**
   * Toggles the intake pistons
   */
  public void toggleIntake(){
    intakePiston.toggle();
  }
}
