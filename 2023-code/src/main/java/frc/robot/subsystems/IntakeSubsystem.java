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

  private final DoubleSolenoid leftPiston;
  private final DoubleSolenoid rightPiston;

  //TODO Maybe an encoder here later

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    leftMotor = new CANSparkMax(IntakeConstants.leftIntakeMotorID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(IntakeConstants.rightIntakeMotorID, MotorType.kBrushless);
    leftPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.leftPistonForwardChannel, IntakeConstants.leftPistonReverseChannel);
    rightPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.rightPistonForwardChannel, IntakeConstants.rightPistonReverseChannel);

    leftMotor.restoreFactoryDefaults();
    leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftMotor.setInverted(false);
    
    rightMotor.restoreFactoryDefaults();
    rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Brings the intake pistons inside the robot
   */
  public void intakeIn() {
    leftPiston.set(Value.kForward);
    rightPiston.set(Value.kForward);
  }

  /**
   * Puts the intake pistons outside the robot
   */
  public void intakeOut() {
    leftPiston.set(Value.kReverse);
    rightPiston.set(Value.kReverse);
  }

  /**
   * Sets the speed of the intake motors by percentage
   * @param speed double value -1.0 to 1.0
   */
  public void setIntakeSpeed(double speed) {
    MathUtil.clamp(speed, -1, 1);
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
}
