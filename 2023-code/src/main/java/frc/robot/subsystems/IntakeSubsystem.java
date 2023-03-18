// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor;

  private static RelativeEncoder intakeEncoder;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    //Constructs and sets up intake motor controllers and solenoids
    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    intakeMotor.restoreFactoryDefaults();

    intakeEncoder = intakeMotor.getEncoder();

    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    intakeMotor.setInverted(false);

    intakeMotor.setSmartCurrentLimit(60);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake spin velocity RPM", intakeEncoder.getVelocity());
  }

  /**
   * Sets the speed of the intake motors by percentage
   * @param speed double value -1.0 to 1.0
   */
  public void setIntakeSpeed(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    intakeMotor.set(speed);
  }

  /**
   * Stops the intake motors
   */
  public void stopIntakeMotors() {
    intakeMotor.stopMotor();
  }

  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }
}
