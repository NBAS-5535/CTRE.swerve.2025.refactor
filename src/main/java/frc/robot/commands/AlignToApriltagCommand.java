package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Vision.LimelightHelpers.RawFiducial;

public class AlignToAprilTagCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_limelight;
  private int m_tagId;

  private  final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, 0.1);
  private  final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.400000, 0.000000, 0.000600, 0.2);
  private  final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.3);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  public double rotationalRate = 0;
  public double velocityX = 0;
  public double velocityY = 0;

  private double baseRotationalRate = 0.5;
  private double baseVelocityX = 0.1;

  public AlignToAprilTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int tagId) {
    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_tagId = tagId;
    addRequirements(m_limelight);
  }


  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    
    RawFiducial fiducial;

    try {
      fiducial = m_limelight.getFiducialWithId(m_tagId);

      rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * 0.75* 0.9;
      
      // final double velocityX = xPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
      velocityX = xPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
      velocityY = yPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
        
      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint()) {
        System.out.println("STOP alignment");
        this.end(true);
      }

      SmartDashboard.putNumber("AlignToAprilTagCommand/txnc", fiducial.txnc);
      SmartDashboard.putNumber("AlignToAprilTagCommand/ta", fiducial.ta);
      SmartDashboard.putNumber("AlignToAprilTagCommand/distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("AlignToAprilTagCommand/distToCamera", fiducial.distToCamera);
      SmartDashboard.putNumber("AlignToAprilTagCommand/rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("AlignToAprilTagCommand/xPidController", velocityX);
      SmartDashboard.putNumber("AlignToAprilTagCommand/TagID", m_tagId);

      /* move the robot to correct position */
      m_drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));
      
    } catch (VisionSubsystem.NoSuchTargetException nste) { 
      //System.out.println("No apriltag found");
      /* if there is no apriltag in sight move the robot until one is found */
      m_drivetrain.setControl(alignRequest.withRotationalRate(this.baseRotationalRate));//.withVelocityX(this.baseVelocityX));
      System.out.println(nste.getMessage());
    }
  }

  @Override
  public boolean isFinished() {
    boolean temp = rotationalPidController.atSetpoint() && xPidController.atSetpoint();
    SmartDashboard.putBoolean("AlignToAprilTagCommand/AlignFinished", temp);
    return temp;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
    
  }
}