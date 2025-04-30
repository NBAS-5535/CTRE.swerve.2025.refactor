// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
//import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
//import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpointHeights;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.subsystems.AlgaeSubsystem.Setpoint;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  // for sim only
  // private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);

  private final SparkFlex m_motor   = new SparkFlex(ElevatorConstants.kElevatorCANId, MotorType.kBrushless);
  private final RelativeEncoder m_encoder  = m_motor.getEncoder();
  
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  
  /*new ProfiledPIDController(ElevatorConstants.kElevatorKp,
                              ElevatorConstants.kElevatorKi,
                              ElevatorConstants.kElevatorKd,
                              new Constraints(ElevatorConstants.kMaxVelocity,
                                              ElevatorConstants.kMaxAcceleration));
   */
 
  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  //private ElevatorState m_state;
  private double m_height;

  /******** Simulation setup */
  // Simulation setup and variables
  private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
  private SparkFlexSim elevatorMotorSim;
  //private SparkLimitSwitchSim elevatorLimitSwitchSim;
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          SimulationRobotConstants.kElevatorGearing,
          SimulationRobotConstants.kCarriageMass,
          SimulationRobotConstants.kElevatorDrumRadius,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          SimulationRobotConstants.kMaxElevatorHeightMeters,
          true,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          0.0,
          0.0);
          // Mechanism2d setup for subsystem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              SimulationRobotConstants.kMinElevatorHeightMeters
                  * SimulationRobotConstants.kPixelsPerMeter,
              90));

  private TrapezoidProfile.State m_trapezoidalGoal;
  private TrapezoidProfile.State m_trapezoidalSetpoint;
  
  /******* Ctor */
  public ElevatorSubsystem() {
    SparkFlexConfig elevatorConfig = new SparkFlexConfig();

    // Configure basic settings of the elevator motor
    elevatorConfig
        .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
        .voltageCompensation(12);
    elevatorConfig.idleMode(IdleMode.kBrake);

    /*
      * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
      * will prevent any actuation of the elevator in the reverse direction if the limit switch is
      * pressed.
      
    elevatorConfig
        .limitSwitch
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen);
    */

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
    elevatorConfig
        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control
        //.pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
        .p(ElevatorConstants.kElevatorKp)
        .outputRange(-1, 1)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(ElevatorConstants.kMaxVelocityInRevolutions)
        .maxAcceleration(ElevatorConstants.kMaxAccelerationInRevolutions)
        .allowedClosedLoopError(0.5);

    m_motor.configure(elevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder.setPosition(0);
    m_trapezoidalSetpoint = new TrapezoidProfile.State(m_encoder.getPosition(), 0);
    m_trapezoidalGoal = m_trapezoidalSetpoint;

    //m_state = ElevatorState.Base;
    m_height = ElevatorConstants.kBaseHeight;

    /******** Simulation setup */
    // Initialize simulation values
    elevatorMotorSim = new SparkFlexSim(m_motor, elevatorMotorModel);
    //elevatorLimitSwitchSim = new SparkLimitSwitchSim(m_motor, false);
      
  }
  
  public enum Setpoint {
    kBase,
    kBottomReef,
    kMiddleReef,
    kAlgaeNet
  }

  public enum ElevatorHeight {
    BaseHeight,
    BottomReefHeight,
    MiddleReefHeight,
    AlgaeNetHeight
  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    m_controller.setReference(
        m_height, ControlType.kMAXMotionPositionControl);
  }
  
  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && m_motor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      m_encoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!m_motor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      m_encoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kBase:
              m_height = ElevatorSetpointHeights.kBaseHeight;
              break;
            case kBottomReef:
              m_height = ElevatorSetpointHeights.kBottomReefHeight;
              break;
            case kMiddleReef:
              m_height = ElevatorSetpointHeights.kMiddleReefHeight;
              break;
            case kAlgaeNet:
              m_height = ElevatorSetpointHeights.kAlgaeNetHeight;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    moveToSetpoint();
    //zeroElevatorOnLimitSwitch();
    //zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Elevator/Target Position", m_height);
    SmartDashboard.putNumber("Elevator/Actual Position", m_encoder.getPosition());

    // Update mechanism2d
    m_elevatorMech2d.setLength(
      SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
          + SimulationRobotConstants.kPixelsPerMeter
              * (m_encoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
              * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    //elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
                    / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                * SimulationRobotConstants.kElevatorGearing)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);
    
    // SimBattery is updated in Robot.java
  }


 
  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
    /*
    double voltsOut = MathUtil.clamp(
        m_controller.calculate(getHeightMeters(), goal) +
        m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
                                              m_controller.getSetpoint().velocity), -7, 7);
    m_motor.setVoltage(voltsOut);
    */
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_motor.set(0.0);
  }

  
}
