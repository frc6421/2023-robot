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

  private DoubleSolenoid grabberPiston;

  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    
    //Constructs the double solenoid for the grabber
    grabberPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, GrabberConstants.FORWARD_CHANNEL, GrabberConstants.REVERSE_CHANNEL);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Extends the grabber piston and closes the grabber
   */
  public void grab(){
    grabberPiston.set(Value.kReverse);
  }

  /**
   * Retracts the grabber piston and opens the grabber
   */
  public void release(){
    grabberPiston.set(Value.kForward);
  }

  /**
   * Toggles the grabber piston
   */
  public void toggleGrabber(){
    grabberPiston.toggle();
  }
}
