// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


public final class Constants {

  
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class ElevatorConstants {

  public static final int kElevator1CanId = 1;
    public static final int kElevator2CanId = 2;

    public static final double kHome = 0;
    public static final double kFeederStation = 14; 
    public static final double kTravel = 0;

    public static final double kLevel0 = 0;
    public static final double kLevel1 = 25;
    public static final double kLevel2 = 10; 
    public static final double kLevel3 = 10; 
    public static final double kLevel4 = 10; 
  }

  public static final class MechanismConstants{

public static final int kFlipper3CanId = 3;

public static final double khome = 5;
public static final double kalgeelevel1 = 5;
public static final double kLevel2 = 5; 
 public static final double kLevel3 = 5; 
 public static final double kLevel4 = 5; 

  }


  public static final class ScoreMotorConstants{

public static final int kScoreMotor4CanId = 4;
public static final int Score_Motor4_CURRENT_LIMIT = 60;
public static final double Score_MOTOR4_VOLTAGE_COMP = 10;
public static final double ScoreMotor4_EJECT_VALUE = 1;


  }
}
