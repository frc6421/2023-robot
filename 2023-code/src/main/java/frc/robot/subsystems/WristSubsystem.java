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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.WristAngleConstants;

public class WristSubsystem extends SubsystemBase
{

    //TODO: !IMPORTANT! SPARKMAX POSITION IN ROTATIONS

    private CANSparkMax wristMotor;
    private SparkMaxPIDController wristPIDController;
    public static RelativeEncoder wristEncoder;


    private double positionMaxOutput; 
    private double positionMinOutput;

    private double wristDynamicFF;

    private double setPoint;

     /** Creates a new WristSubsystem. */
    public WristSubsystem()
    {

        wristMotor = new CANSparkMax(WristConstants.WRIST_CAN_ID, MotorType.kBrushless);

        wristMotor.restoreFactoryDefaults();

        wristEncoder = wristMotor.getEncoder();
        wristEncoder.setPositionConversionFactor(WristConstants.WRIST_DEGREES_PER_MOTOR_ROTATION);

        wristMotor.setSmartCurrentLimit(30);

        wristMotor.setInverted(WristConstants.WRIST_IS_INVERTED);
        wristMotor.setIdleMode(IdleMode.kBrake);

        wristEncoder.setPosition(WristAngleConstants.WRIST_START_POSITION);

        wristPIDController = wristMotor.getPIDController();

        wristPIDController.setFeedbackDevice(wristEncoder);

        setPoint = WristAngleConstants.WRIST_START_POSITION;

        wristMotor.setSoftLimit(SoftLimitDirection.kForward, WristConstants.WRIST_OUT_SOFT_LIMIT);
        wristMotor.setSoftLimit(SoftLimitDirection.kReverse, WristConstants.WRIST_IN_SOFT_LIMIT);
    
        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);


        //// PID ////
        positionMinOutput = -1;
        positionMaxOutput = 1;



        wristPIDController.setP(WristConstants.WRIST_P);
        wristPIDController.setI(WristConstants.WRIST_I);
        wristPIDController.setD(WristConstants.WRIST_D);
        
        wristPIDController.setOutputRange(positionMinOutput, positionMaxOutput);
        
        
    }
    
    @Override
  public void periodic() 
    {
        SmartDashboard.putNumber("Wrist Encoder Angle", getWristDegreePosition());
        
    }


    ////MOTOR AND PID METHODS////
    
    /**
    * Sets motor position to zero
    */
    public void setPosition() 
    {
        wristPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }
    
    
    /**
    * Sets motor to a provided position in degrees
    * @param position The position to set the motor to
    */
    public void 
    setPosition(double position)
    {
        wristPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the wrist to a specified angle
     * @param angle The angle in degrees to set the wrist to
     */
    public void setWristAngleWithGrav(double angle) 
    {
        setGravityOffset();
        wristPIDController.setReference(angle, CANSparkMax.ControlType.kPosition, 0, wristDynamicFF, SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }
    

    /**
     * Takes in and sets an angle and feed forward value. For testing purposes
     * @param angle The angle to set the wrist to
     * @param newFF The feed forward to set the wrist to
    *
     */
    public void setWristAngleAndFF(double angle, double newFF) 
    {
        wristPIDController.setReference(angle, CANSparkMax.ControlType.kPosition, 0 , wristDynamicFF, SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }

    /**
     * Sets the wrist to a specified percent output based on controller input
     * Currently has a max of 15% wrist power
     * @param controllerValue The value passed in by a controller joystick
     */
    public void setPercentWristPower(double controllerValue)
    {
        wristPIDController.setReference(controllerValue * WristConstants.WRIST_MAX_TEST_PERCENT_OUTPUT, CANSparkMax.ControlType.kDutyCycle);
    }

    public void setPercentPosition (double controllerValue)
    {

        double change = controllerValue;

        change = MathUtil.applyDeadband(change, 0.04);

        setPoint = change + setPoint;

        setPoint = MathUtil.clamp(setPoint, (double) WristConstants.WRIST_IN_SOFT_LIMIT, WristConstants.WRIST_OUT_SOFT_LIMIT);
        
        setPosition(setPoint);
    }

    public void setPercentWristPowerNoLimit(double inputValue)
    {
        wristPIDController.setReference(inputValue, CANSparkMax.ControlType.kDutyCycle);
    }

    //TODO: Get rid of, for testing purposes
    public void setGravityOffsetTest()
    {
       wristMotor.set(WristConstants.MAX_WRIST_GRAVITY_FF * Math.cos(Math.toRadians(getWristDegreePosition())));
    }

    public void setGravityOffset()
    {
        wristDynamicFF = (WristConstants.MAX_WRIST_GRAVITY_FF * Math.cos(Math.toRadians(getWristDegreePosition())));
    }

    public double getFeedForward()
    {
        return wristDynamicFF; 
    }

    /**
     * Sets the wrist P value to a specified number
     * @param newP the new P value to be passed in
     */
    public void setWristP(double newP)
    {
        wristPIDController.setP(newP);
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    
    ////ENCODER METHODS////

    /**
     * Returns the position of the wrist in degrees
     * @return Wrist position in degrees
     */
    public double getWristDegreePosition()
    {
        return wristEncoder.getPosition();
    }

    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Resets the encoder position to the start position
     */
    public void resetEncoderPosition() {
        wristEncoder.setPosition(WristAngleConstants.WRIST_START_POSITION);
    }


    ////TEST METHODS////

    
}
