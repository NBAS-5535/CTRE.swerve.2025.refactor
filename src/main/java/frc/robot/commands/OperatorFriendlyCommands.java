// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Pigeon2GyroSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OperatorFriendlyCommands extends Command {
  /** Creates a new OperatorFriendlyCommands. */
  //private double m_initialAngle;
  //private double currentAngle;

  private final CommandSwerveDrivetrain m_swerve;
  private final Pigeon2GyroSubsystem m_pidgy;

  private Pose2d m_initialPose;
  private Pose2d currentPose;
  private String m_commandType = null;

  public OperatorFriendlyCommands(CommandSwerveDrivetrain swerve, Pigeon2GyroSubsystem pigeon, String commandString) {
    this.m_swerve = swerve;
    this.m_pidgy = pigeon;
    this.m_commandType = commandString;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, pigeon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_initialAngle = m_pidgy.getHeading();
    //SmartDashboard.putNumber("Initial Angle", m_initialAngle);
    //( new m_swerve.sysIdRotate(Direction.kForward).withTimeout(10));
    m_pidgy.setAngleMarker();

    m_initialPose = m_swerve.getCurrentPose();
    SmartDashboard.putNumber("Initial X", m_initialPose.getX());
    SmartDashboard.putNumber("Initial Y", m_initialPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_commandType) {
      case "rotate":
        //currentAngle = m_pidgy.getHeading();
        //SmartDashboard.putNumber("Current Angle", currentAngle);
        //SmartDashboard.putNumber("Diff Angle", currentAngle - m_initialAngle);
        //System.out.println(getName() + String.valueOf(currentAngle) + "  " + String.valueOf(m_initialAngle));
        break;
      case "pose":
        currentPose = m_swerve.getCurrentPose();
        SmartDashboard.putNumber("Current X", currentPose.getX());
        SmartDashboard.putNumber("Current Y", currentPose.getY());
        break;
      default:
        System.out.println(getName() + " Unknowm/Unimplemented commandType");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Boolean stopIt = false;
    /*
    if (m_commandType == "rotate" || m_commandType == "pose"){
      stopIt = true;
    }
      */
    if ( m_commandType != null) {
      stopIt = true;
    }
    return stopIt;
  }
  /*
  private Boolean angleDiffReached() {
    return Math.abs(currentAngle) - Math.abs(m_initialAngle) >= 90.;
  }
    */
  
}
