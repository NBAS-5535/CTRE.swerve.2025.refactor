// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon2GyroSubsystem extends SubsystemBase {
  /** Creates a new Pigen2GyroSubsystem. */
  private final Pigeon2 m_pigeon2;
  private final Pigeon2SimState m_pigeon2SimState; 

  private double m_initialAngle = 0.;
  private double m_initialPose;
  private double m_angleDiff;

  // instead of stopping motors
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public Pigeon2GyroSubsystem(Pigeon2 pigeon2) {
    this.m_pigeon2 = pigeon2;
    this.m_pigeon2SimState = m_pigeon2.getSimState();
    
    zeroHeading();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Yaw", getHeading());
    SmartDashboard.putNumber("Angle", getGyroAngle().getDegrees());
    m_angleDiff = getHeading() - m_initialAngle;
  }

  public void zeroHeading(){
    m_pigeon2.reset();
  }

  public double getHeading(){
    return m_pigeon2.getYaw().getValueAsDouble();
  }

  public Rotation2d getGyroAngle(){
    return m_pigeon2.getRotation2d();
  }

  public void setAngleMarker(){
    m_initialAngle = getHeading();
    SmartDashboard.putNumber("AngleSet", m_initialAngle);
  }

  public boolean isAngleDiffReached(CommandSwerveDrivetrain swerve, double maxAngle) {
    //System.out.println(getName() + ": " + String.valueOf(m_angleDiff));
    //boolean condition = MathUtil.isNear(maxAngle, Math.abs(m_angleDiff), 1.);
    boolean condition = Math.abs(m_angleDiff) >= 0.935 * maxAngle;
    if ( condition ) {
      //swerve.stopAllMotors();
      swerve.applyRequest(() -> idleRequest);
      SmartDashboard.putBoolean("isAngleDiffReached", condition);
      //System.out.println(getName() + " ---- AngleDiffReached"); 
    }
    return condition;
  }
}
