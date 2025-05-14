// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /* Some tolerance settings */
  public static final double distanceEpsilon = 0.01;

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 12;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 10;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 11;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 9;

    public static final boolean kGyroReversed = false;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final String LIMELIGHT_NAME = "";

    // height of the center of the Limelight lens from the floor (inches)
    public static final double limelightLensHeightInches = 10.;
    // height of the target from the floor (inches)
    public static final double targetHeightInches = 33.5;

    // calibration distance for correct mount angle
    public static final double knownDistance = 10.;

    public static final int testTagId = 5;
  }

    /* **************************
   * ActuatorSubsystemConstants
   */
  public static final class ActuatorSubsystemConstants {
    public static final int kActuatorMotorCanId = 44;

    public static double kActuatorRampRate = 0.1;
    public static int    kActuatorCurrentLimit = 40;
    public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);

    public static final double kActuatorKp = 0.1;
    public static final double kActuatorKi = 0;
    public static final double kActuatorKd = 0.;
    
    public static final double ActuatorSetpointTestSpeed = 0.1;
  
    public static final double kSpeed = 0.1; //<---- set smaller as a test, should be 0.5ish
    public static final double kIntermediateSetPoint = 30.; //revolutions
    public static final double kSetPointInRevolutions = 39.; //revolutions <---- set smaller as a test, should be 40ish
    
    public static final class ActuatorSubSystemSetpoints {
      public static final double kBase = 0;
      public static final double kAlgaeNetShootSetPoint = 30.;
      public static final double kSetPointInRevolutions = 39.;
     }

  }

  /* **************************
   * AlgaeSubsystemConstants
   */
  public static final class AlgaeSubsystemConstants {
    public static final int kElevatorMotorCanId = 42;
    public static final int kArmMotorCanId = 41;
    public static final int kIntakeMotorCanId = 43;

    public static final double ElevatorSetpointTestSpeed = 0.1;
    public static final double ArmSetpointTestSpeed = 0.2;
  
    public static final double ElevatorSetpointThreshold = 0.5;
    public static final double ArmSetpointThreshold = 0.5;

    public static final class ElevatorSetpoints {
      public static final double kGroundPick = 26.04; //44.72;
      public static final double kSideShoot = 41.91; //61.85;
      public static final double kBase = 0.;
      public static final double kCorralDrop = 32.63; //73.43;
      public static final double kShootAfterCorralDrop = 39.7; //73.43;
      public static final double kMoveWithBall = -6.; //-10.0;
      public static final double kClearReefLevels = -5.;
      public static final double kLowerReef = -4.78; //-1.5; //-8.57;
      public static final double kHigherReef = -46.91; //-83.00;
      public static final double kAlgaeNet = -148.15; //-264; //-248.5;
    }

    public static final class ArmSetpoints {
      public static final double kGroundPick = -17.15;
      public static final double kSideShoot = 20.61;
      public static final double kBase = 0.;
      public static final double kCorralDrop = 14.33;
      public static final double kShootAfterCorralDrop = 6.28;
      public static final double kMoveWithBall = -17.15;
      public static final double kClearReefLevels = 22.;
      public static final double kLowerReef = 12.0;//15.0;//18.67;
      public static final double kHigherReef = 12.0;//15.0;//18.67;
      public static final double kAlgaeNet = 31.9;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.7; // power-level for motor
      public static final double kReverse = -0.7; // reverse
    }
  }

  /* *********************
   * LifSubsystemConstants
   */
  public static final class LiftSubsystemConstants{
    public static final int kLiftMotorCanId = 45;

    public static double kLiftRampRate = 0.1;
    public static int    kLiftCurrentLimit = 40;
    public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);

    public static final double kLiftKp = 0.1;
    public static final double kLiftKi = 0;
    public static final double kLiftKd = 0.;

    public static final double LiftSetpointTestSpeed = 0.5;
    // Lift control parameters
    public static final class LiftSubSystemSetpoints {
      public static final double kBase = 0;
      public static final double kmaxLiftSetpoint = 200.;
     }

  }

  /* *********************
   * AutonomousConstants
   */
  public static final class AutonomousMenuConstants{
    public static final String kDownBlue = "Start_Down_Blue/Blue1";
    public static final String kCenterBlue = "Start_Center_Blue/Blue2";
    public static final String kUpBlue = "Start_Up_Blue/Blue3";

    public static final String kDownRed = "Start_Down_Red/Red1";
    public static final String kCenterRed = "Start_Center_Red/Red2";
    public static final String kUpRed = "Start_Up_Red/Red3";
  }

  public static final class AutonomousModeOptions{
    public static final String kCorralOnly = "Corral-Only";
    public static final String kCorralPlusAlgae = "Corral-And-HighReef-Pickup";
  }
}