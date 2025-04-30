// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ActuatorSubsystemRev;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActuatorCommandRev extends Command {
  /** Creates a new test. */
  private ActuatorSubsystemRev m_actuator;
  private int m_direction;
  private double m_setpoint;

  public ActuatorCommandRev(ActuatorSubsystemRev actuator, int direction, double setpoint) {
    this.m_actuator = actuator;
    this.m_direction = direction;
    this.m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_actuator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_actuator.markPosition();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_actuator.setInMotion(m_direction);
    m_actuator.setCurrentPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_actuator.isSetpointReached(m_direction, m_setpoint);
  }
}
