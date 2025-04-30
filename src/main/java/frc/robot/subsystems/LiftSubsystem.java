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
import frc.robot.Constants.LiftSubsystemConstants;
import frc.robot.Constants.LiftSubsystemConstants.LiftSubSystemSetpoints;



public class LiftSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum LiftSetpoints {
    kBase,
    kmaxSetpoint
  }

  // Initialize Lift SPARK. We will use MAXMotion position control for the Lift, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax liftMotor =
      new SparkMax(LiftSubsystemConstants.kLiftMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController liftController = liftMotor.getClosedLoopController();
  private RelativeEncoder liftEncoder = liftMotor.getEncoder();

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private double LiftCurrentTarget = LiftSubSystemSetpoints.kBase; // may have to start at a heigher level kMiddleReef

  // manual jog flag
  private boolean runPeriodic = false;
  
  public LiftSubsystem() {
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
    liftMotor.configure(
        Configs.LiftSubsystem.liftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    // Zero Lift and elevator encoders on initialization
    liftEncoder.setPosition(0);

  }

  /**
   * Drive the Lift and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  public void moveToSetpoint() {
    liftController.setReference(LiftCurrentTarget, ControlType.kMAXMotionPositionControl);

  }

  /** Zero the Lift and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      liftEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  /** Set Lift motor power in the range of [-1, 1]. - TEST Purpose: step through */
  private void setLiftPower(double power) {
    liftMotor.set(power);
  }

  /**
   * Command to set the subsystem setpoint. This will set the Lift and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(LiftSetpoints setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kmaxSetpoint:
              LiftCurrentTarget = LiftSubSystemSetpoints.kmaxLiftSetpoint;
              break;
            case kBase:
              LiftCurrentTarget = LiftSubSystemSetpoints.kBase;
              break;

          }
        });
  }


  /**
   * Command to run the lift motor. 
   * Intended to step through to adjust proper setpoints for elevator heights
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runLiftUpCommand() {
    return this.startEnd(
        () -> this.setLiftPower(LiftSubsystemConstants.LiftSetpointTestSpeed), 
        () -> this.setLiftPower(0.0));
  }

  public Command runLiftDownCommand() {
    return this.startEnd(
        () -> this.setLiftPower((-1) * LiftSubsystemConstants.LiftSetpointTestSpeed), 
        () -> this.setLiftPower(0.0));
  }

  @Override
  public void periodic() {
    // manual jog option: don't run moveToSetpoint
    if (runPeriodic) {
      moveToSetpoint();
    }
    //zeroElevatorOnLimitSwitch();
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Lift/Target Position", LiftCurrentTarget);
    SmartDashboard.putNumber("Lift/Actual Position", liftEncoder.getPosition());

    
  }

  @Override
  public void simulationPeriodic() {
  }


}