package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ActuatorSubsystemConstants;
import frc.robot.Constants.ActuatorSubsystemConstants.ActuatorSubSystemSetpoints;



public class ActuatorSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum ActuatorSetpoints {
    kBase,
    kAlgaeNetShootSetPoint,
    kSetPointInRevolutions
  }

  // Initialize Actuator SPARK. We will use MAXMotion position control for the Actuator, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax actuatorMotor =
      new SparkMax(ActuatorSubsystemConstants.kActuatorMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController actuatorController = actuatorMotor.getClosedLoopController();
  private RelativeEncoder actuatorEncoder = actuatorMotor.getEncoder();

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private double ActuatorCurrentTarget = ActuatorSubSystemSetpoints.kBase; // may have to start at a heigher level kMiddleReef

  
  public ActuatorSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    actuatorMotor.configure(
        Configs.ActuatorSubsystem.actuatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    // Zero Actuator and elevator encoders on initialization
    actuatorEncoder.setPosition(0);

  }

  /**
   * Drive the Actuator and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  public void moveToSetpoint() {
    actuatorController.setReference(ActuatorCurrentTarget, ControlType.kMAXMotionPositionControl);

  }

  /** Zero the Actuator and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      actuatorEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  /** Set Actuator motor power in the range of [-1, 1]. - TEST Purpose: step through */
  private void setActuatorPower(double power) {
    actuatorMotor.set(power);
  }

  /**
   * Command to set the subsystem setpoint. This will set the Actuator and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(ActuatorSetpoints setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kAlgaeNetShootSetPoint:
              ActuatorCurrentTarget = ActuatorSubSystemSetpoints.kAlgaeNetShootSetPoint;
              break;
            case kSetPointInRevolutions:
              ActuatorCurrentTarget = ActuatorSubSystemSetpoints.kSetPointInRevolutions;
              break;
            case kBase:
              ActuatorCurrentTarget = ActuatorSubSystemSetpoints.kBase;
              break;

          }
        });
  }


  /**
   * Command to run the actuator motor. 
   * Intended to step through to adjust proper setpoints for elevator heights
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runActuatorUpCommand() {
    return this.startEnd(
        () -> this.setActuatorPower(ActuatorSubsystemConstants.ActuatorSetpointTestSpeed), 
        () -> this.setActuatorPower(0.0));
  }

  public Command runActuatorDownCommand() {
    return this.startEnd(
        () -> this.setActuatorPower((-1) * ActuatorSubsystemConstants.ActuatorSetpointTestSpeed), 
        () -> this.setActuatorPower(0.0));
  }

  public Command runActuatorToAlgaeNetCommand() {
    return this.startEnd(
        () -> this.setActuatorPower((-1) * ActuatorSubsystemConstants.ActuatorSetpointTestSpeed), 
        () -> this.setActuatorPower(0.0));
  }

  public boolean isSetpointReached(double setpoint){
    SmartDashboard.putNumber("ActuatorCurrentTarget", ActuatorCurrentTarget);
    double currentPosition = actuatorEncoder.getPosition();
    SmartDashboard.putNumber("ActuatorPosition", currentPosition);
    boolean condition = Math.abs(currentPosition - ActuatorCurrentTarget) <= 0.5;
    SmartDashboard.putBoolean("Actuator-isSetpointReached", condition);
    if ( condition ) {
      setActuatorPower(0.);
      System.out.println("Actuator Motor Stopped: ");
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    moveToSetpoint();
    //zeroElevatorOnLimitSwitch();
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Actuator/Target Position", ActuatorCurrentTarget);
    SmartDashboard.putNumber("Actuator/Actual Position", actuatorEncoder.getPosition());

    
  }

  @Override
  public void simulationPeriodic() {
  }


}