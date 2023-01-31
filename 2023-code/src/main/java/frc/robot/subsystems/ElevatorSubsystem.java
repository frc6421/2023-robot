package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    
    //TODO IMPORTANT: SPARK MAX POSITION IN ROTATIONS

    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController elevatorPIDController;
    public static RelativeEncoder elevatorEncoder;

    private double positionMaxOutput; 
    private double positionMinOutput;
    
    private double elevatorPositionInMeters;

    private ShuffleboardTab elevatorTab;
    private GenericEntry elevatorPositionEntry;
    private GenericEntry elevatorVolatgeEntry;
    private GenericEntry elevatorPEntry;

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem()
    {
        elevatorMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);

        elevatorMotor.restoreFactoryDefaults();

        elevatorMotor.setInverted(true);

        elevatorEncoder = elevatorMotor.getEncoder();

        // Degrees per motor rotation
        elevatorEncoder.setPositionConversionFactor(ElevatorConstants.ELEVATOR_SPROCKET_PITCH_CIRCUMFERENCE / ElevatorConstants.ELEVATOR_GEAR_RATIO);
        elevatorEncoder.setPosition(0); //TODO Verify start position
    
        elevatorMotor.setIdleMode(IdleMode.kBrake); 

        // Set soft limits
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, ElevatorConstants.ELEVATOR_FORWARD_SOFT_LIMIT_METERS);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, ElevatorConstants.ELEVATOR_REVERSE_SOFT_LIMIT);

        elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

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

        elevatorTab = Shuffleboard.getTab("Elevator Tab");

        elevatorPositionEntry = elevatorTab.add("Encoder Position: ", 0) 
            .getEntry();

        elevatorVolatgeEntry = elevatorTab.add("Elevator Voltage: ", 0)
            .getEntry();

        elevatorPEntry = elevatorTab.add("Elevator P: ", 0)
            .getEntry();
        
    }

    @Override
    public void periodic()
    {
        elevatorPositionEntry.setDouble(getElevatorEncoderPosition());
        elevatorVolatgeEntry.setDouble(elevatorMotor.getBusVoltage());
        elevatorPEntry.setDouble(elevatorPIDController.getP());
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
    }

    /**
     * Sets the controler to move with a certin max percentage
     * Current max is 15%
     * @param controlerValue
     */
    public void setElevatorWithPercent(double controlerValue)
    {
        elevatorMotor.set(controlerValue * ElevatorConstants.ELEVATOR_MAX_PRECENT);
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
}
