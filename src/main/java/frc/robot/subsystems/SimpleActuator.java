// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SimpleActuator extends SubsystemBase {

  public enum ActuatorSetpoints {
    kBase,
    kAlgaeNetShootSetPoint
  }
  public static final int kActuatorMotorCanId = 44;
  public static double kActuatorRampRate = 0.1;
  public static int    kActuatorCurrentLimit = 40;
  public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
  public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
  public static final double kActuatorKp = 0.1;
  public static final double kActuatorKi = 0;
  public static final double kActuatorKd = 0.;

  private SparkMax actuatorMotor = new SparkMax(kActuatorMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController actuatorController = actuatorMotor.getClosedLoopController();
  private RelativeEncoder actuatorEncoder = actuatorMotor.getEncoder();

  public static final SparkMaxConfig actuatorConfig = new SparkMaxConfig();
 // static 
 {
      // Configure basic setting of the actuator motor
      actuatorConfig
        .smartCurrentLimit(kActuatorCurrentLimit)
        .closedLoopRampRate(kActuatorRampRate);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      actuatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .pid(kActuatorKp, 
              kActuatorKi, 
              kActuatorKd)
          .outputRange(-1,1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.25);

      actuatorConfig.idleMode(IdleMode.kBrake);

      actuatorMotor.configure(
        actuatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    }



  /** Creates a new SimpleActuator. */
  public SimpleActuator() {
    actuatorEncoder.setPosition(0);
  }

  public void movetoposition(double revos){
    actuatorEncoder.setPosition(revos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
