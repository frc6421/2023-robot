// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {

  private DoubleSolenoid leftGrabberPiston;
  private DoubleSolenoid rightGrabberPiston;

  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    //Constructs the double solenoid for the grabber
    leftGrabberPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, GrabberConstants.LEFT_FORWARD_CHANNEL, GrabberConstants.LEFT_REVERSE_CHANNEL);
    rightGrabberPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, GrabberConstants.RIGHT_FORWARD_CHANNEL, GrabberConstants.RIGHT_REVERSE_CHANNEL);
    grab();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Extends the grabber piston and closes the grabber
   */
  public void grab(){
    leftGrabberPiston.set(Value.kForward);
    rightGrabberPiston.set(Value.kForward);
  }

  /**
   * Retracts the grabber piston and opens the grabber
   */
  public void release(){
    leftGrabberPiston.set(Value.kReverse);
    rightGrabberPiston.set(Value.kReverse);
  }

  /**
   * Toggles the grabber piston
   */
  public void toggleGrabber(){
    leftGrabberPiston.toggle();
    rightGrabberPiston.toggle();
  }
}
