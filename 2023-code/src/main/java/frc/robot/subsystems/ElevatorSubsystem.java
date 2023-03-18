package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController elevatorPIDController;
    public static RelativeEncoder elevatorEncoder;

    private double positionMaxOutput; 
    private double positionMinOutput;
    
    private double elevatorPositionInMeters;

    private double setPoint;

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem()
    {
        elevatorMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);

        elevatorMotor.restoreFactoryDefaults();

        elevatorMotor.setInverted(true);

        elevatorMotor.setSmartCurrentLimit(30);
        
        elevatorEncoder = elevatorMotor.getEncoder();

        // Degrees per motor rotation
        elevatorEncoder.setPositionConversionFactor(ElevatorConstants.ELEVATOR_SPROCKET_PITCH_CIRCUMFERENCE / ElevatorConstants.ELEVATOR_GEAR_RATIO);
        elevatorEncoder.setPosition(0);
    
        elevatorMotor.setIdleMode(IdleMode.kCoast);

        
        // Set soft limits
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, ElevatorConstants.ELEVATOR_FORWARD_SOFT_LIMIT_METERS);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, ElevatorConstants.ELEVATOR_REVERSE_SOFT_LIMIT);

        elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        setPoint = ElevatorConstants.ELEVATOR_REVERSE_SOFT_LIMIT;

        // PID \\
        positionMaxOutput = 1; 
        positionMinOutput = -1;

        elevatorPIDController = elevatorMotor.getPIDController();

        elevatorPIDController.setFeedbackDevice(elevatorEncoder);

        elevatorPIDController.setP(ElevatorConstants.ELEVATOR_P, 0);
        elevatorPIDController.setI(ElevatorConstants.ELEVATOR_I, 0);
        elevatorPIDController.setD(ElevatorConstants.ELEVATOR_D, 0);
        elevatorPIDController.setFF(ElevatorConstants.ELEVATOR_FF, 0);
        elevatorPIDController.setOutputRange(positionMinOutput, positionMaxOutput, 0);
    }

    @Override
    public void periodic()
    {
        //SmartDashboard.putNumber("elevator pos", getElelvatorPositionInMeters());
    }

    // MOTOR PID \\
    public void setElevatorPosition() 
    {
        elevatorPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }
    
    public void setP(double value)
    {
        elevatorPIDController.setP(value, 0);
    }

    /**
     * Sets position to given input
     * @param position the position to set motor to
     */
    public void setElevatorPosition(double position)
    {
        elevatorPIDController.setReference(position, CANSparkMax.ControlType.kPosition, 0, 0.02375, SparkMaxPIDController.ArbFFUnits.kPercentOut);
        // elevatorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the controler to move with a certin max percentage
     * Current max is 15%
     * @param FF
     */
    public void goToPosition(double FF)
    {
        elevatorMotor.set(FF);
    } 
    
    public void setPercentPosition (double controllerValue)
    {
        controllerValue = MathUtil.applyDeadband(controllerValue, 0.04);

        double change = controllerValue * 0.02;

        setPoint = (change + setPoint);

        setPoint = MathUtil.clamp(setPoint, (double) ElevatorConstants.ELEVATOR_REVERSE_SOFT_LIMIT, ElevatorConstants.ELEVATOR_FORWARD_SOFT_LIMIT_METERS);
        
        setElevatorPosition(setPoint);
    }

    // ENCODER \\
    public double getElelvatorPositionInMeters()
    {
        elevatorPositionInMeters = getElevatorEncoderPosition()*ElevatorConstants.ELEVATOR_SPROCKET_PITCH_CIRCUMFERENCE;
        
        return elevatorPositionInMeters;
    }

    public static double getElevatorEncoderPosition()
    {
        return elevatorEncoder.getPosition();
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

}
