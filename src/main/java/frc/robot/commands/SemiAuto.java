package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ActuatorSubsystemConstants;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.Setpoint;

public class SemiAuto {

      /* */
  /* Game action commands */
  public static Command runAlgaePickupLowerReefCommand(AlgaeSubsystem algae) {
    return new SequentialCommandGroup(
        algae.setSetpointCommand(Setpoint.kAlgaePickupLowerReef),
        new InstantCommand(() -> algae.moveToSetpoint())
       );
  }

  public static Command runAlgaePickupHigherReefCommand(AlgaeSubsystem algae) {
    return new SequentialCommandGroup(
        algae.setSetpointCommand(Setpoint.kAlgaePickupHigherReef),
        new InstantCommand(() -> algae.moveToSetpoint())
       );
  }
  public static Command runGroundPickupCommand(AlgaeSubsystem algae) {
    return new SequentialCommandGroup(
        algae.setSetpointCommand(Setpoint.kGroundPickup),
        new InstantCommand(() -> algae.moveToSetpoint())
       );
  }
  public static Command runShootAlgaeNetCommand(AlgaeSubsystem algae) {
    return new SequentialCommandGroup(
        algae.setSetpointCommand(Setpoint.kShootAlgaeNet),
        new InstantCommand(() -> algae.moveToSetpoint())
       );
  }

  public static Command runSideSlotShootCommand(AlgaeSubsystem algae, ActuatorSubsystem actuator) {
    return null;
  }
}
