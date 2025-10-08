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

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, 0.1);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.400000, 0.000000, 0.000600, 0.2);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.3);
  
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  public double rotationalRate = 0.1;
  public double velocityX = 0.1;
  //private RawFiducial fiducial; 
  private double targetAreaTolerance = 0.5;

  public FindAprilTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int tagId) {
    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_tagId = tagId;
    SmartDashboard.putNumber("FindAprilTag/TagToFindIn", tagId);
    SmartDashboard.putNumber("FindAprilTag/TagToFind", m_tagId);
    addRequirements(drivetrain, m_limelight);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    
    try {
      RawFiducial fiducial = m_limelight.getFiducialWithId(m_tagId);

      rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * 0.75* 0.9;
      
      final double velocityX = xPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
        
      //if (fiducial.ta > 0 ) { //} || 
      // check if there is a valid target
      //if ( m_limelight.getTV() ) {  
      // check if there is a valid target AND the robot is near the Setpoints
      if (fiducial.ta > 0 && (rotationalPidController.atSetpoint() && xPidController.atSetpoint())) {
      //if (isFinished()) {
        System.out.println("Found Tag");
        this.end(true);
      }

      SmartDashboard.putNumber("FindAprilTag/TagID", m_tagId);
      SmartDashboard.putNumber("FindAprilTag/txnc", fiducial.txnc);
      SmartDashboard.putBoolean("FindAprilTag/Found", m_limelight.getTV());
      SmartDashboard.putNumber("FindAprilTag/distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("FindAprilTag/distToCamera", fiducial.distToCamera);
      SmartDashboard.putNumber("FindAprilTag/area", fiducial.ta);
      SmartDashboard.putNumber("FindAprilTag/rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("FindAprilTag/velocityX", velocityX);
      
      /* uncomment for action */
      m_drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));

    } catch (VisionSubsystem.NoSuchTargetException nste) { 
      //System.out.println(getName() + " - No apriltag found");
      if ((rotationalRate != 0) && (velocityX != 0)){
        /* move the robot until apriltag is found??? */
        m_drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));
        /* original statement
          alignRequest.withRotationalRate(-rotationalRate).withVelocityX(-velocityX));//.withVelocityY(velocityY));
        */
      } else {
        m_drivetrain.setControl(alignRequest.withRotationalRate(0.3).withVelocityX(0.1));
      }
    }
  }

  @Override
  public boolean isFinished() {
    boolean t1 = rotationalPidController.atSetpoint();
    boolean t2 = xPidController.atSetpoint();
    //boolean temp = t1 && t2;
    // or need the following??
    boolean temp = m_limelight.getTV() && t1 && t2;
    //System.out.println("rotPID: " + String.valueOf(t1) + " - xPID: " + String.valueOf(t2));
    SmartDashboard.putBoolean("FindAprilTag/AlignFinished", temp);
    return temp;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("FindAprilTagCommand - Ended");
    m_drivetrain.applyRequest(() -> idleRequest);
    
  }

  public void setAprilTagId(int tagId){
    m_tagId = tagId;
  }
}