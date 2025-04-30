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

  public static final class CoralSubsystemConstants {
    public static final int kElevatorMotorCanId = 4;
    public static final int kArmMotorCanId = 3;
    public static final int kIntakeMotorCanId = 2;

    public static final class ElevatorSetpoints {
      public static final int kFeederStation = 0;
      public static final int kLevel1 = 0;
      public static final int kLevel2 = 0;
      public static final int kLevel3 = 100;
      public static final int kLevel4 = 150;
    }

    public static final class ArmSetpoints {
      public static final double kFeederStation = 33;
      public static final double kLevel1 = 0;
      public static final double kLevel2 = 2;
      public static final double kLevel3 = 2;
      public static final double kLevel4 = 19;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.5;
      public static final double kReverse = -0.5;
    }
  }

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

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }

  public static final class VisionConstants {
    public static final String LIMELIGHT_NAME = "";

    // height of the center of the Limelight lens from the floor (inches)
    public static final double limelightLensHeightInches = 3.5;
    // height of the target from the floor (inches)
    public static final double targetHeightInches = 33.5;

    // calibration distance for correct mount angle
    public static final double knownDistance = 10.;

    public static final int testTagId = 3;
  }

  public static final class ElevatorConstants {
    public static int kElevatorCANId = 42;

    public static double kElevatorRampRate = 0.1;
    public static int    kElevatorCurrentLimit = 40;
    public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static double kMaxVelocityInRevolutions = 4200; 
    public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static double kMaxAccelerationInRevolutions = 6000;

    public static final double kElevatorKp = 0.1;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0.;

    public static final double kElevatorkS = 0.01964; // volts (V)
    public static final double kElevatorkV = 3.894; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.173; // volt per acceleration (V/(m/s²))
    public static final double kElevatorkG = 0.5; // volts (V)

    public static final double kBaseHeight = 0.0;
    public static final double kBottomReefHeight = 5.0;
    public static final double kMiddleReefHeight = 10.0;
    public static final double kAlgaeNetHeight = 15.0;
    
    public static final class ElevatorSetpointHeights {
      public static final double kBaseHeight = 0.0;
      public static final double kBottomReefHeight = 5.0;
      public static final double kMiddleReefHeight = 10.0;
      public static final double kAlgaeNetHeight = 15.0;
    }

    
  }

  /* *****************
   * ActuatorConstants
   
  public static final class ActuatorConstants{
    public static final int kActuatorMotorCanId = 44;

    public static double kActuatorRampRate = 0.1;
    public static int    kActuatorCurrentLimit = 40;
    public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);

    public static final double kActuatorKp = 0.1;
    public static final double kActuatorKi = 0;
    public static final double kActuatorKd = 0.;

    // actuator control parameters
    public static final double kSpeed = 0.1; //<---- set smaller as a test, should be 0.5ish
    public static final double kIntermediateSetPoint = 15.; //revolutions
    public static final double kSetPointInRevolutions = 39.; //revolutions <---- set smaller as a test, should be 40ish

  }
    */

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