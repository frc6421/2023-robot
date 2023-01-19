package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{
    
    private CANSparkMax armMotor;
    private SparkMaxPIDController armPIDController;
    public static RelativeEncoder armEncoder;


    private double positionMaxOutput; 
    private double positionMinOutput;

     /** Creates a new ArmSubsystem. */
    public ArmSubsystem()
    {

        armMotor = new CANSparkMax(ArmConstants.ARM_CAN_ID, MotorType.kBrushless); //TODO: Get CANID

        armMotor.restoreFactoryDefaults();

        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);

    
        armMotor.setIdleMode(IdleMode.kBrake);

        armPIDController = armMotor.getPIDController();
  
        armPIDController.setFeedbackDevice(armEncoder);

        armMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.ARM_SOFT_LIMIT);
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, -ArmConstants.ARM_SOFT_LIMIT);
    
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        //// PID ////
        positionMaxOutput = 1; 
        positionMinOutput = -1;

        armPIDController.setP(ArmConstants.ARM_P);
        armPIDController.setI(ArmConstants.ARM_I);
        armPIDController.setD(ArmConstants.ARM_D);
        armPIDController.setFF(ArmConstants.ARM_FF);
        
        armPIDController.setOutputRange(positionMinOutput, positionMaxOutput);
    }
    
    @Override
  public void periodic() 
    {
        // Might be needed to update FF based on angle
    }
    /**
    * Sets motor position to zero
    */
    public void setPosition() 
    {
        armPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }
    
    /**
    * Sets motor to a provided position
    * @param position The position to set the motor to
    */
    public void setPosition(double position)
    {
        armPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void setArmAngle(double angle) 
    {
        armPIDController.setReference(angle / ArmConstants.DEGREES_PER_MOTOR_ROTATION, CANSparkMax.ControlType.kPosition);
    }

    public static double getArmEncoderPosition()
    {
        return armEncoder.getPosition();
    }
}
