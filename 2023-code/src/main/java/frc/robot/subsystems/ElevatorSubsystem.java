package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    
    //TODO IMPORTANT: SPARK MAX POSITION IN ROTATIONS

    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController elevatorPIDController;
    public static RelativeEncoder elevatorEncoder;

    private double positionMaxOutput; 
    private double positionMinOutput;  

      /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem()
    {
        elevatorMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);

        elevatorMotor.restoreFactoryDefaults();

        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPosition(0); //TODO Verify start position
    
        elevatorMotor.setIdleMode(IdleMode.kBrake); 
  
        elevatorPIDController.setFeedbackDevice(elevatorEncoder);

        // Set soft limits
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, ElevatorConstants.ELEVATOR_SOFT_LIMIT);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, -ElevatorConstants.ELEVATOR_SOFT_LIMIT);

        elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        // PID \\
        positionMaxOutput = 1; 
        positionMinOutput = -1;

        elevatorPIDController = elevatorMotor.getPIDController();

        elevatorPIDController.setP(ElevatorConstants.ELEVATOR_P);
        elevatorPIDController.setI(ElevatorConstants.ELEVATOR_I);
        elevatorPIDController.setD(ElevatorConstants.ELEVATOR_D);
        elevatorPIDController.setFF(ElevatorConstants.ELEVATOR_FF);
        elevatorPIDController.setOutputRange(positionMinOutput, positionMaxOutput);
    }

    @Override
    public void periodic()
    {
        // May be used later
    }

    // MOTOR PID \\
    public void setPosition() 
    {
        elevatorPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets position to given input
     * @param position the position to set motor to
     */
    public void setPosition(double position)
    {
        elevatorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the controler to move with a certin max percentage
     * Current max is 15%
     * @param controlerValue
     */
    public void setElevatorWithPercent(double controlerValue)
    {
        elevatorPIDController.setReference(controlerValue * ElevatorConstants.ELEVATOR_MAX_PRECENT, CANSparkMax.ControlType.kVelocity);
    
        SmartDashboard.putNumber("SetPoint", controlerValue);
        SmartDashboard.putNumber("ProcessVariable", elevatorEncoder.getVelocity());
    } 

    // ENCODER \\
    public static double getElevatorEncoderPosition()
    {
        return elevatorEncoder.getPosition();
    }
}
