// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of algae project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeSubsystemConstants;
import frc.robot.Constants.AlgaeSubsystemConstants.IntakeSetpoints;
import frc.robot.subsystems.AlgaeSubsystem;


public class ManualCommands {

    public static Command runElevatorUpCommand(AlgaeSubsystem algae) {
        return algae.startEnd(
            () -> algae.setElevatorPower(AlgaeSubsystemConstants.ElevatorSetpointTestSpeed), 
            () -> algae.setElevatorPower(0.0));
    }

    public static Command runElevatorDownCommand(AlgaeSubsystem algae) {
        return algae.startEnd(
            () -> algae.setElevatorPower((-1) * AlgaeSubsystemConstants.ElevatorSetpointTestSpeed), 
            () -> algae.setElevatorPower(0.0));
    }

    /**
     * Command to run the elevator motor. 
     * Intended to step through to adjust proper setpoints for elevator heights
     * When the command is interrupted, e.g. the button is released, the motor will stop.
     */
    public static Command runArmUpCommand(AlgaeSubsystem algae) {
        return algae.startEnd(
            () -> algae.setArmPower(AlgaeSubsystemConstants.ArmSetpointTestSpeed), 
            () -> algae.setArmPower(0.0));
    }

    public static Command runArmDownCommand(AlgaeSubsystem algae) {
        return algae.startEnd(
            () -> algae.setArmPower((-1) * AlgaeSubsystemConstants.ArmSetpointTestSpeed), 
            () -> algae.setArmPower(0.0));
    }

    /**
     * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
     * the motor will stop.
     */
    public static Command runIntakeCommand(AlgaeSubsystem algae) {
        return algae.startEnd(
            () -> algae.setIntakePower(IntakeSetpoints.kForward), () -> algae.setIntakePower(0.0));
    }

    /**
     * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
     * released, the motor will stop.
     */
    public static Command reverseIntakeCommand(AlgaeSubsystem algae) {
        return algae.startEnd(
            () -> algae.setIntakePower(IntakeSetpoints.kReverse), () -> algae.setIntakePower(0.0));
    }

    /**
     * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
     * released, the motor will stop.
     */
    public static Command reverseIntakeCommandSlow(AlgaeSubsystem algae) {
        return algae.startEnd(
            () -> algae.setIntakePower(-0.4), () -> algae.setIntakePower(0.0));
    }
}