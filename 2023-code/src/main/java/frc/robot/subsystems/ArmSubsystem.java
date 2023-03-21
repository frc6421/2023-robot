package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmAngleConstants;

public class ArmSubsystem extends SubsystemBase
{

    //TODO: !IMPORTANT! SPARKMAX POSITION IN ROTATIONS

    private CANSparkMax armMotor;
    private SparkMaxPIDController armPIDController;
    public static RelativeEncoder armEncoder;


    private double positionMaxOutput; 
    private double positionMinOutput;

    private double armDynamicFF;

    private double setPoint;

     /** Creates a new ArmSubsystem. */
    public ArmSubsystem()
    {

        armMotor = new CANSparkMax(ArmConstants.ARM_CAN_ID, MotorType.kBrushless); //TODO: Get CANID

        armMotor.restoreFactoryDefaults();

        armEncoder = armMotor.getEncoder();
        armEncoder.setPositionConversionFactor(ArmConstants.DEGREES_PER_MOTOR_ROTATION); //TODO: Verify this is not totally wrong

        armMotor.setInverted(ArmConstants.ARM_IS_INVERTED);
        armMotor.setIdleMode(IdleMode.kBrake);

        armEncoder.setPosition(ArmAngleConstants.ARM_START_POSITION); //TODO: Verify start position

        armPIDController = armMotor.getPIDController();

        armPIDController.setFeedbackDevice(armEncoder);

        setPoint = ArmAngleConstants.ARM_START_POSITION;

        armMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.ARM_OUT_SOFT_LIMIT);
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.ARM_IN_SOFT_LIMIT);
    
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);


        //// PID ////
        positionMinOutput = -1;
        positionMaxOutput = 1;



        armPIDController.setP(ArmConstants.ARM_P);
        armPIDController.setI(ArmConstants.ARM_I);
        armPIDController.setD(ArmConstants.ARM_D);
        
        armPIDController.setOutputRange(positionMinOutput, positionMaxOutput);
        
    }
    
    @Override
  public void periodic() 
    {
        SmartDashboard.putNumber("Arm Encoder Angle", getArmDegreePosition());
    }


    ////MOTOR AND PID METHODS////
    
    /**
    * Sets motor position to zero
    */
    public void setPosition() 
    {
        armPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }
    
    
    /**
    * Sets motor to a provided position in degrees
    * @param position The position to set the motor to
    */
    public void 
    setPosition(double position)
    {
        armPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the arm to a specified angle
     * @param angle The angle in degrees to set the arm to
     */
    public void setArmAngleWithGrav(double angle) 
    {
        setGravityOffset();
        armPIDController.setReference(angle, CANSparkMax.ControlType.kPosition, 0, armDynamicFF, SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }
    

    /**
     * Takes in and sets an angle and feed forward value. For testing purposes
     * @param angle The angle to set the arm to
     * @param newFF The feed forward to set the arm to
    *
     */
    public void setArmAngleAndFF(double angle, double newFF) 
    {
        armPIDController.setReference(angle, CANSparkMax.ControlType.kPosition, 0 , armDynamicFF, SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }

    /**
     * Sets the arm to a specified percent output based on controller input
     * Currently has a max of 15% arm power
     * @param controllerValue The value passed in by a controller joystick
     */
    public void setPercentArmPower(double controllerValue)
    {
        armPIDController.setReference(controllerValue * ArmConstants.ARM_MAX_TEST_PERCENT_OUTPUT, CANSparkMax.ControlType.kDutyCycle);
    }

    public void setPercentPosition (double controllerValue)
    {

        double change = controllerValue;

        change = MathUtil.applyDeadband(change, 0.04);

        setPoint = change + setPoint;

        setPoint = MathUtil.clamp(setPoint, (double) ArmConstants.ARM_IN_SOFT_LIMIT, ArmConstants.ARM_OUT_SOFT_LIMIT);
        
        setPosition(setPoint);
    }

    public void setPercentArmPowerNoLimit(double inputValue)
    {
        armPIDController.setReference(inputValue, CANSparkMax.ControlType.kDutyCycle);
    }

    //TODO: Get rid of, for testing purposes
    public void setGravityOffsetTest()
    {
       armMotor.set(ArmConstants.MAX_ARM_GRAVITY_FF * Math.cos(Math.toRadians(getArmDegreePosition())));
    }

    public void setGravityOffset()
    {
        armDynamicFF = (ArmConstants.MAX_ARM_GRAVITY_FF * Math.cos(Math.toRadians(getArmDegreePosition())));
    }

    public double getFeedForward()
    {
        return armDynamicFF; 
    }

    /**
     * Sets the arm P value to a specified number
     * @param newP the new P value to be passed in
     */
    public void setArmP(double newP)
    {
        armPIDController.setP(newP);
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    
    ////ENCODER METHODS////

    /**
     * Returns the position of the arm in degrees
     * @return Arm position in degrees
     */
    public double getArmDegreePosition()
    {
        return armEncoder.getPosition();
    }

    public double getSetPoint() {
        return setPoint;
    }


    ////TEST METHODS////

    
}
