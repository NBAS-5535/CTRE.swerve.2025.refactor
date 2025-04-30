package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeSubsystemConstants;
import frc.robot.Constants.AlgaeSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.AlgaeSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.AlgaeSubsystemConstants.IntakeSetpoints;

public class AlgaeSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kGroundPickup,
    kClearWires,
    kClearReef,
    kSideSlotShoot,
    kBase,
    kCorralDrop,
    kShootCorralDrop,
    kMoveWithBall,
    kAlgaePickupLowerReef,
    kAlgaePickupHigherReef,
    kShootAlgaeNet
  }

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkFlex armMotor =
      new SparkFlex(AlgaeSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkFlex elevatorMotor =
      new SparkFlex(AlgaeSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  private SparkMax intakeMotor =
      new SparkMax(AlgaeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private double armCurrentTarget = ArmSetpoints.kBase; // may have to start at a heigher level kMiddleReef
  private double elevatorCurrentTarget = ElevatorSetpoints.kBase;

  // manual jog flag
  private boolean runPeriodic = true;
  /*
  // Create the SysId routine: Data collection
  SysIdRoutine elevatorSysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      (voltage) -> elevatorMotor.setVoltage(voltage.in(Volts)),
      null, // No log consumer, since data is recorded by URCL
      this
    )
  );
  */


  public AlgaeSubsystem() {
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
    armMotor.configure(
        Configs.AlgaeSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorMotor.configure(
        Configs.AlgaeSubsystem.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeMotor.configure(
        Configs.AlgaeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero arm and elevator encoders on initialization
    armEncoder.setPosition(0);
    elevatorEncoder.setPosition(0);


  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  public void moveToSetpoint() {
    armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      armEncoder.setPosition(0);
      elevatorEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  /** Set elevator motor power in the range of [-1, 1]. - TEST Purpose: step through */
  public void setElevatorPower(double power) {
    elevatorMotor.set(power);
  }

  /** Set arm motor power in the range of [-1, 1]. - TEST Purpose: step through */
  public void setArmPower(double power) {
    armMotor.set(power);
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kGroundPickup:
              armCurrentTarget = ArmSetpoints.kGroundPick;
              elevatorCurrentTarget = ElevatorSetpoints.kGroundPick;
              break;
            case kClearWires:
              armCurrentTarget = ArmSetpoints.kGroundPick;
              elevatorCurrentTarget = ElevatorSetpoints.kMoveWithBall;
              break;
            case kClearReef:
              armCurrentTarget = ArmSetpoints.kClearReefLevels;
              elevatorCurrentTarget = ElevatorSetpoints.kClearReefLevels;
              break;
            case kSideSlotShoot:
              armCurrentTarget = ArmSetpoints.kSideShoot;
              elevatorCurrentTarget = ElevatorSetpoints.kSideShoot;
              break;
            case kBase:
              armCurrentTarget = ArmSetpoints.kBase;
              elevatorCurrentTarget = ElevatorSetpoints.kBase;
              break;
            case kCorralDrop:
              armCurrentTarget = ArmSetpoints.kCorralDrop;
              elevatorCurrentTarget = ElevatorSetpoints.kCorralDrop;
              break;
            case kShootCorralDrop:
              armCurrentTarget = ArmSetpoints.kShootAfterCorralDrop;
              elevatorCurrentTarget = ElevatorSetpoints.kShootAfterCorralDrop;
              break;
            case kMoveWithBall:
              armCurrentTarget = ArmSetpoints.kMoveWithBall;
              elevatorCurrentTarget = ElevatorSetpoints.kMoveWithBall;
              break;
            case kAlgaePickupLowerReef:
              armCurrentTarget = ArmSetpoints.kLowerReef;
              elevatorCurrentTarget = ElevatorSetpoints.kLowerReef;
              break;
            case kAlgaePickupHigherReef:
              armCurrentTarget = ArmSetpoints.kHigherReef;
              elevatorCurrentTarget = ElevatorSetpoints.kHigherReef;
              break;
            case kShootAlgaeNet:
              armCurrentTarget = ArmSetpoints.kAlgaeNet;
              elevatorCurrentTarget = ElevatorSetpoints.kAlgaeNet;
              break;
          }
        });
  }

  /**
   * Command to run the elevator motor. 
   * Intended to step through to adjust proper setpoints for elevator heights
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runElevatorUpCommand() {
    runPeriodic = false;
    return this.startEnd(
        () -> this.setElevatorPower(AlgaeSubsystemConstants.ElevatorSetpointTestSpeed), 
        () -> this.setElevatorPower(0.0));
  }

  public Command runElevatorDownCommand() {
    runPeriodic = false;
    return this.startEnd(
        () -> this.setElevatorPower((-1) * AlgaeSubsystemConstants.ElevatorSetpointTestSpeed), 
        () -> this.setElevatorPower(0.0));
  }

  /**
   * Command to run the elevator motor. 
   * Intended to step through to adjust proper setpoints for elevator heights
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runArmUpCommand() {
    runPeriodic = false;
    return this.startEnd(
        () -> this.setArmPower(AlgaeSubsystemConstants.ArmSetpointTestSpeed), 
        () -> this.setArmPower(0.0));
  }

  public Command runArmDownCommand() {
    runPeriodic = false;
    return this.startEnd(
        () -> this.setArmPower((-1) * AlgaeSubsystemConstants.ArmSetpointTestSpeed), 
        () -> this.setArmPower(0.0));
  }

  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0));
  }

  public Command runIntakeCommandSlow() {
    return this.startEnd(
        () -> this.setIntakePower(0.4), () -> this.setIntakePower(0.0));
  }

  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command reverseIntakeCommandSlow() {
    return this.startEnd(
        () -> this.setIntakePower(-0.4), () -> this.setIntakePower(0.0));
  }

  /**
   * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   */
  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
  }

  @Override
  public void periodic() {
    if ( runPeriodic ) {
      moveToSetpoint();
    }

    //zeroElevatorOnLimitSwitch();
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Arm/Target Position", armCurrentTarget);
    SmartDashboard.putNumber("Arm/Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator/Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Applied Output", intakeMotor.getAppliedOutput());
    SmartDashboard.putBoolean("runPeriodic", runPeriodic);

    
  }

  /**
   * Method to keep track of arm/elevator states in terms of the setpoins 
   * 
   */
  public double[] getSetpointDetails(Setpoint setpoint){
    double[] setpointDetails = new double[]{0., 0.};
    switch (setpoint) {
      case kGroundPickup:
        setpointDetails[0] = ArmSetpoints.kGroundPick;
        setpointDetails[1] = ElevatorSetpoints.kGroundPick;
        break;
      case kClearWires:
        setpointDetails[0] = ArmSetpoints.kGroundPick;
        setpointDetails[1] = ElevatorSetpoints.kMoveWithBall;
        break;
      case kClearReef:
        setpointDetails[0] = ArmSetpoints.kClearReefLevels;
        setpointDetails[1] = ElevatorSetpoints.kClearReefLevels;
        break;
      case kSideSlotShoot:
        setpointDetails[0] = ArmSetpoints.kSideShoot;
        setpointDetails[1] = ElevatorSetpoints.kSideShoot;
        break;
      case kBase:
        setpointDetails[0] = ArmSetpoints.kBase;
        setpointDetails[1] = ElevatorSetpoints.kBase;
        break;
      case kCorralDrop:
        setpointDetails[0] = ArmSetpoints.kCorralDrop;
        setpointDetails[1] = ElevatorSetpoints.kCorralDrop;
        break;
      case kShootCorralDrop:
        setpointDetails[0] = ArmSetpoints.kShootAfterCorralDrop;
        setpointDetails[1] = ElevatorSetpoints.kShootAfterCorralDrop;
        break;
      case kMoveWithBall:
        setpointDetails[0] = ArmSetpoints.kMoveWithBall;
        setpointDetails[1] = ElevatorSetpoints.kMoveWithBall;
        break;
      case kAlgaePickupLowerReef:
        setpointDetails[0] = ArmSetpoints.kLowerReef;
        setpointDetails[1] = ElevatorSetpoints.kLowerReef;
        break;
      case kAlgaePickupHigherReef:
        setpointDetails[0] = ArmSetpoints.kHigherReef;
        setpointDetails[1] = ElevatorSetpoints.kHigherReef;
        break;
      case kShootAlgaeNet:
        setpointDetails[0] = ArmSetpoints.kAlgaeNet;
        setpointDetails[1] = ElevatorSetpoints.kAlgaeNet;
        break;
    }
    return setpointDetails;
  }

  /**
   * Method to compare current encoder reading with the setpoins 
   * May be used to ensure the arm/elevator motion is not prematurely terminated
   */
  public boolean isSetpointReached(double armTarget, double elevatorTarget){
    double armCurrentPosition = armEncoder.getPosition();
    double elevatorCurrentPosition = elevatorEncoder.getPosition();
    if ( ( Math.abs(armCurrentPosition) >= Math.abs(armTarget) - Constants.distanceEpsilon) &&
         ( Math.abs(elevatorCurrentPosition) >= Math.abs(elevatorTarget) - Constants.distanceEpsilon) ) {
          return true;
         }
    return false;
  }

  public void resetPeriodicMode() {
    runPeriodic = true;
  }
  
  public void setPeriodicToFalse() {
    runPeriodic = false;
  }

  @Override
  public void simulationPeriodic() {
  }


  /* sysid: data collection Commands 
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.dynamic(direction);
  }
    */

}