// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ActuatorSubsystemConstants;
import frc.robot.Constants.AlgaeSubsystemConstants;

public class ActuatorSubsystemRev extends SubsystemBase {
  /** Creates a new ActuatorSubsystem. */
  private SparkMax actuatorMotor =
      new SparkMax(ActuatorSubsystemConstants.kActuatorMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController actuatorController = actuatorMotor.getClosedLoopController();
  private RelativeEncoder actuatorEncoder = actuatorMotor.getEncoder();

  private double initialPosition;
  private double currentPosition;

  public ActuatorSubsystemRev() {
    /* moved into Config.java 
    SparkMaxConfig actuatorConfig = new SparkMaxConfig();

    actuatorConfig
        .smartCurrentLimit(ActuatorConstants.kActuatorCurrentLimit)
        .closedLoopRampRate(ActuatorConstants.kActuatorRampRate)
        .closedLoop
        .pid(ActuatorConstants.kActuatorKp, ActuatorConstants.kActuatorKi, ActuatorConstants.kActuatorKd)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
        actuatorConfig.idleMode(IdleMode.kBrake);
      */

    actuatorMotor.configure(
        Configs.ActuatorSubsystem.actuatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero actuator encoder on initialization
    this.setPosition(0);

    // sets max revolutions in Constants.java as reference -> moves the actuator to that position (i.e., upright) 
    //actuatorController.setReference(ActuatorConstants.kSetPointInRevolutions, ControlType.kPosition);
    // set reference as the fully-retracted position
    actuatorController.setReference(0, ControlType.kPosition);

    initialPosition = actuatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setInMotion();
    SmartDashboard.putNumber("Actuator Position", actuatorEncoder.getPosition());
  }

  public void setPosition(double position){
    actuatorEncoder.setPosition(position);
  }
  
  public double getPosition(){
    return actuatorEncoder.getPosition();
  }

  public void setInMotion(int direction) {
    actuatorMotor.set(direction * ActuatorSubsystemConstants.ActuatorSetpointTestSpeed);
    SmartDashboard.putNumber("Actuator Speed", ActuatorSubsystemConstants.ActuatorSetpointTestSpeed);
  }

  public void setCurrentPosition(){
    currentPosition = getPosition();
    SmartDashboard.putNumber("setCurPos", currentPosition);
  }

  public void stopMotor() {
    actuatorMotor.set(0.);
  }

  /**
   * mark the current position of the actuator
   *
   * @param none
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command markPositionCommand() {
    //initialPosition = getPosition();
    //SmartDashboard.putNumber("Actuator Init", initialPosition);
    return this.runOnce( () -> this.markPosition());
  }

  // plain-vanilla marking
  public void markPosition() {
    initialPosition = getPosition();
    SmartDashboard.putNumber("Actuator Init", initialPosition);
  }

  /**
   * check if the actuator reached setpoint
   *
   * @param none 
   * @return true if setpoint is reached
   */
  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public boolean isReachedSetpoint(int direction) {
    double currentPosition = actuatorEncoder.getPosition();
    double [] temp = {initialPosition, currentPosition, ActuatorSubsystemConstants.ActuatorSubSystemSetpoints.kSetPointInRevolutions};
    SmartDashboard.putNumberArray("Actuator Positions", temp);
    boolean condition = direction * currentPosition >= direction * initialPosition + ActuatorSubsystemConstants.ActuatorSubSystemSetpoints.kSetPointInRevolutions;
    SmartDashboard.putBoolean("isReachedSetpoint", condition);
    if ( condition ) {
      stopMotor();
      System.out.println("Motor Stopped: " + String.valueOf(direction));
      return true;
    } else {
      return false;
    }
  }

  public boolean isSetpointReached(int direction, double setpoint){
    SmartDashboard.putNumber("initial_position", initialPosition);
    SmartDashboard.putNumber("current_position", currentPosition);
    SmartDashboard.putNumber("actuator_setpoint", setpoint);

    boolean condition = direction * currentPosition >= direction * initialPosition + setpoint;
    SmartDashboard.putBoolean("Actuator-isSetpointReached", condition);
    if ( condition ) {
      stopMotor();
      System.out.println("Motor Stopped: " + String.valueOf(direction));
      return true;
    } else {
      return false;
    }
  }
}
