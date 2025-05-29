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

/* Already defined in AlignCommand.java file
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
*/

public class FindAprilTagCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_limelight;
  private int m_tagId;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, 0.01);
    private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  public double rotationalRate = 0.1;
  private RawFiducial fiducial; 
  private double targetAreaTolerance = 0.5;

  public FindAprilTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int tagId) {
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
    
    try {
      if (m_tagId == 0) {
        fiducial = m_limelight.getClosestFiducial();
      } else {
        fiducial = m_limelight.getFiducialWithId(m_tagId);
      }

      rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * 0.75* 0.9;

      /* check if a valid target was found */
      if ( fiducial.ta > targetAreaTolerance ) { //&& rotationalPidController.atSetpoint()) {
        this.end(true);
      }

      SmartDashboard.putNumber("FindAprilTag/txnc", fiducial.txnc);
      SmartDashboard.putNumber("FindAprilTag/distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("FindAprilTag/area", fiducial.ta);
      SmartDashboard.putNumber("FindAprilTag/rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("FindAprilTag/TagID", m_tagId);
      /* uncomment for action */
      m_drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate));

    } catch (VisionSubsystem.NoSuchTargetException nste) { 
      System.out.println(getName() + " - No apriltag found");
      if (rotationalRate != 0){
        /* uncomment for action */
        m_drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate));
      }  else {
        /* if there is no apriltag in sight move the robot until one is found */
        // TO DO !!!!
        m_drivetrain.setControl(alignRequest.withRotationalRate(-0.05));
      }
    }
  }

  @Override
  public boolean isFinished() {
    boolean temp = fiducial.ta > targetAreaTolerance; //rotationalPidController.atSetpoint();
    SmartDashboard.putBoolean("FindAprilTag/FoundAprilTag", temp);
    return temp;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
    
  }
}