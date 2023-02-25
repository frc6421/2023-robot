// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax intakeArmMotor;

  private SparkMaxPIDController intakeArmPIDController;
  private static RelativeEncoder intakeArmEncoder;

  private double positionMaxOutput; 
  private double positionMinOutput; 

  private double setPoint;

  private double intakeArmDynamicFF;

  private ShuffleboardTab intakeTab;
  private GenericEntry intakeAngle;
  private GenericEntry intakeFF;

  //TODO Maybe an encoder here later

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    //Constructs and sets up intake motor controllers and solenoids
    intakeMotor = new CANSparkMax(IntakeConstants.GRAB_INTAKE_MOTOR_ID, MotorType.kBrushless);

    intakeArmMotor = new CANSparkMax(IntakeConstants.ARM_INTAKE_MOTOR_ID, MotorType.kBrushless);

    intakeMotor.restoreFactoryDefaults();
    intakeArmMotor.restoreFactoryDefaults();

    intakeArmEncoder = intakeArmMotor.getEncoder();

    intakeArmEncoder.setPositionConversionFactor(IntakeConstants.DEGREES_PER_MOTOR_ROTATION);

    intakeArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    intakeMotor.setInverted(false);
    intakeArmMotor.setInverted(false);

    intakeMotor.setSmartCurrentLimit(60);
    intakeArmMotor.setSmartCurrentLimit(50);

    intakeArmPIDController = intakeArmMotor.getPIDController();

    intakeArmPIDController.setFeedbackDevice(intakeArmEncoder);

    intakeArmEncoder.setPosition(IntakeConstants.INTAKE_UP_ANGLE);

    intakeArmMotor.setSoftLimit(SoftLimitDirection.kForward, IntakeConstants.INTAKE_UP_SOFT_LIMIT);
    intakeArmMotor.setSoftLimit(SoftLimitDirection.kReverse, IntakeConstants.INTAKE_BOTTOM_SOFT_LIMIT);

    intakeArmMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    intakeArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    intakeArmPIDController.setP(IntakeConstants.INTAKE_ARM_P);
    intakeArmPIDController.setI(IntakeConstants.INTAKE_ARM_I);
    intakeArmPIDController.setD(IntakeConstants.INTAKE_ARM_D);
    
    positionMinOutput = -1;
    positionMaxOutput = 1;

    intakeArmPIDController.setOutputRange(positionMinOutput, positionMaxOutput);

    setPoint = IntakeConstants.INTAKE_UP_SOFT_LIMIT;

    intakeTab = Shuffleboard.getTab("Intake Tab");

    intakeAngle = intakeTab.add("Intake Angle: ", 0)
      .getEntry();
    
    intakeFF = intakeTab.add("Intake FF", 0)
      .getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeAngle.setDouble(getIntakeArmDegreePosition());
    intakeFF.setDouble(intakeArmPIDController.getFF());
  }


  /**
   * Sets the arm to a specified angle
   * @param angle The angle in degrees to set the arm to
   */
  public void setIntakeArmAngleWithGrav(double angle) 
  {
        setGravityOffset();
        intakeArmPIDController.setReference(angle, CANSparkMax.ControlType.kPosition, 0, intakeArmDynamicFF, SparkMaxPIDController.ArbFFUnits.kPercentOut);
  }
  /**
   * Sets the speed of the intake motors by percentage
   * @param speed double value -1.0 to 1.0
   */
  public void setIntakeSpeed(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    intakeMotor.set(speed);
  }

  public void setGravityOffset()
  {
      intakeArmDynamicFF = (IntakeConstants.MAX_ARM_GRAVITY_FF * Math.cos(Math.toRadians(getIntakeArmDegreePosition())));
  }

  public double getIntakeArmDegreePosition()
  {
      return intakeArmEncoder.getPosition();
  }

  public void setIntakeArmP(double newP)
  {
      intakeArmPIDController.setP(newP);
  }

  public void setSetPoint(double setPoint) {
      this.setPoint = setPoint;
  }

  public double getFeedForward()
  {
      return intakeArmDynamicFF; 
  }
  public void setIntakeArmFF(double FF)
  {
    intakeArmPIDController.setFF(FF);
  }
  public void setPercentPosition (double controllerValue)
  {
      double change = controllerValue;

      change = MathUtil.applyDeadband(change, 0.04);

      setPoint = change + setPoint;

      setPoint = MathUtil.clamp(setPoint, (double) IntakeConstants.INTAKE_FLOOR_ANGLE, IntakeConstants.INTAKE_UP_ANGLE);
      
      setPosition(setPoint);
  }
  public void setPosition(double position)
  {
      intakeArmPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Stops the intake motors
   */
  public void stopIntakeMotors(){
    intakeMotor.stopMotor();
  }

  
  public double getSetPoint() {
    return setPoint;
  }

  public void setArmIntakeSpeed(double speed){
    intakeArmMotor.set(speed);
  }

}
