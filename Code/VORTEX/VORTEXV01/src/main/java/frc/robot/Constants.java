// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = 50; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(4)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class AutonConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);  /// AUTOS DO DIA 08/04 ESTAVAM COM 1
    public static final PIDConstants ANGLE_PID       = new PIDConstants(10, 0, 0); //0.04
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class Field {
    public static final double CORAL_L1 = 7.5;
    public static final double CORAL_L2 = 11.5; //40.80  10.25
    public static final double CORAL_L3 = 18.5;   //69.33    17.36
    public static final double CORAL_L4 = 26.75;

    public static final double ALGA_L2 = 6.25;  // 25
    public static final double ALGA_L3 = 11.8;  // 52
  }
  
  public static class Robot_Elevator{
    public static final DigitalInput sensOuttake = new DigitalInput(0);
    public static final SparkMax mOutInc = new SparkMax(20, MotorType.kBrushless);
    public static final SparkMaxConfig configOutInc = new SparkMaxConfig();
    public static SparkClosedLoopController pidControllerOutInc;

    public static final SparkMax mPrincipal = new SparkMax(21, MotorType.kBrushless);
    public static final SparkMaxConfig configElevator = new SparkMaxConfig();
    public static SparkClosedLoopController pidControllerElevator;

    public static final SparkMax mOuttake = new SparkMax(23, MotorType.kBrushless);
    public static final SparkMaxConfig configOuttake = new SparkMaxConfig();
    public static SparkClosedLoopController pidControllerOuttake;
  }

  public static class Robot_Chassy{
    public static final Pigeon2 pigeon = new Pigeon2(13);
    public static final DigitalInput sensIntake = new DigitalInput(1);
  }

  public static class Robot_Suspension{
    public static final SparkMax mPrincipal = new SparkMax(19, MotorType.kBrushless);
    public static final SparkMaxConfig configSuspension = new SparkMaxConfig();
    public static SparkClosedLoopController pidController;
  }

  public static class Robot_Intake{
    public static final SparkMax mFunil = new SparkMax(22, MotorType.kBrushless);
    public static final SparkMaxConfig configFunil = new SparkMaxConfig();
    public static SparkClosedLoopController pidControllerFunil;

    public static final SparkMax mColeta = new SparkMax(18, MotorType.kBrushless);
    public static final SparkMaxConfig configColeta = new SparkMaxConfig();
    public static SparkClosedLoopController pidControllerColeta;

    public static final SparkMax mInclina = new SparkMax(16, MotorType.kBrushless);
    public static final SparkMaxConfig configInclina = new SparkMaxConfig();
    public static SparkClosedLoopController pidControllerInclina;
  }
  
}

/*
 * Inclinação do Outtake max = -8,95
 * Inclinação do Outtake L4 = -8.2
 * 
 * Inlcinação do Outtake Coral Station = -50
 * 
 */
