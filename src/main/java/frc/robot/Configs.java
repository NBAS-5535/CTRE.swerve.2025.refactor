// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants.ActuatorSubsystemConstants;
import frc.robot.Constants.LiftSubsystemConstants;


/** Add your docs here. */
public class Configs {

 
   /* *****************
   * ActuatorSubsystem 
   */
  public static final class ActuatorSubsystem {
    public static final SparkMaxConfig actuatorConfig = new SparkMaxConfig();

    static {
      // Configure basic setting of the actuator motor
      actuatorConfig
        .smartCurrentLimit(ActuatorSubsystemConstants.kActuatorCurrentLimit)
        .closedLoopRampRate(ActuatorSubsystemConstants.kActuatorRampRate);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      actuatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .pid(ActuatorSubsystemConstants.kActuatorKp, 
              ActuatorSubsystemConstants.kActuatorKi, 
              ActuatorSubsystemConstants.kActuatorKd)
          .outputRange(-1,1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.25);

      actuatorConfig.idleMode(IdleMode.kBrake);
    }
  }

  /* *****************
   * AlgaeSubsystem 
   */
  public static final class AlgaeSubsystem {
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();
    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      /* 
       *Configure basic settings of the arm motor 
       */
      armConfig
          .idleMode(IdleMode.kBrake).smartCurrentLimit(40)
          .voltageCompensation(12);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.25);

      /* 
       * Configure basic settings of the elevator motor
       */
      elevatorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      elevatorConfig
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);

      /* 
       * Configure basic settings of the intake motor
       * */
      intakeConfig
          .inverted(true)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40);
    }
  }

  /* *****************
   * LiftSubsystem 
   */
  public static final class LiftSubsystem {
    public static final SparkMaxConfig liftConfig = new SparkMaxConfig();

    static {
      // Configure basic setting of the lift motor
      liftConfig
        .smartCurrentLimit(LiftSubsystemConstants.kLiftCurrentLimit)
        .closedLoopRampRate(LiftSubsystemConstants.kLiftRampRate);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      liftConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .pid(LiftSubsystemConstants.kLiftKp, 
               LiftSubsystemConstants.kLiftKi, 
               LiftSubsystemConstants.kLiftKd)
          .outputRange(-1., 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.25);

      liftConfig.idleMode(IdleMode.kBrake);
    }
  }
  /* *****************
   * VisionSubsystem 
   */
  public static final class VisionSubsystem {
    /* the filed layout json is located at
     * https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/
     */
    private final String jsonPath = "Vision/2025-reefscape-welded.json";
    //public final AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2025Reefscape.m_resourceFile);
  }
}