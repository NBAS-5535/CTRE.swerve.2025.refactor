package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import org.w3c.dom.views.DocumentView;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Vision.LimelightHelpers.RawFiducial;
import frc.robot.Constants.DriveTrainConstants;
//import frc.robot.Vision.LimelightHelpers.LimelightTarget_Fiducial;
//import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.controller.PIDController;


class PIDControllerConfigurable extends PIDController {
  public PIDControllerConfigurable(double kP, double kI, double kD) {
      super(kP, kI, kD);
  }
  
  public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
      super(kP, kI, kD);
      this.setTolerance(tolerance);
  }
}
public class AlignCommand extends Command {
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

  public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int tagId) {
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
    //LimelightTarget_Fiducial targetFiducial; 

    try {
      if (m_tagId == 0) {
        fiducial = m_limelight.getClosestFiducial();
        //targetFiducial = m_limelight.getTargetFiducialWithId(m_tagId);
      } else {
        fiducial = m_limelight.getFiducialWithId(m_tagId);
        //targetFiducial = m_limelight.getTargetFiducialWithId(m_tagId);
        //System.out.println("got it! " + String.valueOf(m_tagId));
      }

      rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * 0.75* 0.9;
      
      //final double velocityX = xPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
      velocityX = xPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
        
      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint()) {
        System.out.println("STOP alignment");
        this.end(true);
      }

      SmartDashboard.putNumber("AlignCommand/txnc", fiducial.txnc);
      SmartDashboard.putNumber("AlignCommand/ta", fiducial.ta);
      SmartDashboard.putNumber("AlignCommand/distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("AlignCommand/distToCamera", fiducial.distToCamera);
      SmartDashboard.putNumber("AlignCommand/rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("AlignCommand/xPidController", velocityX);
      SmartDashboard.putNumber("AlignCommand/TagID", m_tagId);


      /* move the robot to correct position */
      m_drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));
      
    } catch (VisionSubsystem.NoSuchTargetException nste) { 
      //System.out.println("No apriltag found");
      if ((rotationalRate != 0) && (velocityX != 0)){
        /* move the robot until apriltag is found??? */
        m_drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));
        /* original statement
          alignRequest.withRotationalRate(-rotationalRate).withVelocityX(-velocityX));//.withVelocityY(velocityY));
        */
      } else {
        /* if there is no apriltag in sight move the robot until one is found */
        // TO DO !!!!
        m_drivetrain.setControl(alignRequest.withRotationalRate(-0.05));//.withVelocityX(0.2));
        // simply end the command to stop the robot
        //this.end(true);
      }
    }
  }

  @Override
  public boolean isFinished() {
    boolean temp = rotationalPidController.atSetpoint() && xPidController.atSetpoint();
    SmartDashboard.putBoolean("AlignCommand/AlignFinished", temp);
    return temp;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
    
  }
}