// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class BlinkinSubsystem extends SubsystemBase {
  private static boolean purpled;
  private static Spark blinkinLED = new Spark(9); // 9 is the RIO PWM port this is connected to
  /** Creates a new BlinkinSubsystem. */
  public BlinkinSubsystem() {
    purpled = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public static void blinkinRainbowSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_RAINBOW);
  }
  
  public static void blinkinRedSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_RED);
  }
  
  public static void blinkinBlueSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_BLUE);
  }

  public static void blinkinYellowSet(){
    purpled = false;
    blinkinLED.set(BlinkinConstants.BLINKIN_YELLOW);
  }

  public static void blinkinVioletSet(){
    purpled = true;
    blinkinLED.set(BlinkinConstants.BLINKIN_VIOLET);
  }
  
  public static void blinkinGreenSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_GREEN);
  }
  public static void blinkinHotPinkSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_HOT_PINK);
  }
  
  public static void blinkinRainbowWaveSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_RAINBOW_WAVE);
  }
  
  public static void blinkinRainbowSinelonSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_RAINBOW_SINELON);
  }
  
  public static void blinkinConfettiSet() {
    blinkinLED.set(BlinkinConstants.BLINKIN_CONFETTI);
  }
  
  // Fades from color set on device into black
  public static void blinkinFadeToBlack(){
    blinkinLED.set(BlinkinConstants.BLINKIN_FADE_TO_BLACK);
  }

  public static void blinkinFireSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_FIRE);
  }

  public static void blinkinGlitterSet(){
    blinkinLED.set(BlinkinConstants.BLINKIN_GLITTER);
  }

  public static void blinkinPartyWaveSet()
  {
    blinkinLED.set(BlinkinConstants.BLINKIN_PARTY_WAVE);
  }

  public static void blinkinShotRedSet()
  {
    blinkinLED.set(BlinkinConstants.BLINKIN_SHOT_RED);
  }

  public static boolean isPurpled(){
    return purpled;
  }



  

  /** Gets the last PWM value set to the Blinkin */
  public static double getBlinkinColor(){
    return blinkinLED.get();
  }

  public static void blinkinRandomSet()
  {
    int randValue = (int) (Math.random() * 7);

    if (randValue == 0)
    {
      blinkinConfettiSet();
    }

    if (randValue == 1)
    {
      blinkinRainbowSet();
    }

    if (randValue == 2)
    {
      blinkinRainbowSinelonSet();
    }

    if (randValue == 3)
    {
      blinkinGlitterSet();
    }

    if (randValue == 4)
    {
      blinkinFireSet();
    }
    
    if (randValue == 5)
    {
      blinkinPartyWaveSet();
    }

    if (randValue == 6)
    {
      blinkinShotRedSet();
    }
  }

}


