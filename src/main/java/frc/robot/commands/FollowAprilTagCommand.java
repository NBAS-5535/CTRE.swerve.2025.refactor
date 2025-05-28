package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.PIDControllerConfigurable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Vision.LimelightHelpers.LimelightTarget_Fiducial;

public class FollowAprilTagCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;
  private final int apriltagToFollow;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(
      2.5, 0, 0, 0.1);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.7, 0, 0, 0.02);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.7, 0, 0, 0.02);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  // private static final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();

  public FollowAprilTagCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, int id) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.apriltagToFollow = id;
    // addRequirements(this.drivetrain, this.limelight);
  }

  @Override
  public void initialize() {
    //System.out.println("FollowAprilTag_COMMAND Started");
  }

  @Override
  public void execute() {
    LimelightTarget_Fiducial fiducial;
    SmartDashboard.putNumber("FollowAprilTag/TEST", 0);
    SmartDashboard.putNumber("FollowAprilTag/Id", apriltagToFollow);
    try {
      fiducial = limelight.getTargetFiducialWithId(apriltagToFollow);
      Pose3d targetPoseRobotSpace = fiducial.getTargetPose_RobotSpace();
      double distToRobot = targetPoseRobotSpace.getZ();
      double rotationalError = targetPoseRobotSpace.getRotation().getY();
      // double idk = fiducial.getTargetPose_RobotSpace().getY();
      SmartDashboard.putNumber("FollowAprilTag/TEST", distToRobot);
      // SmartDashboard.putNumber("idk", idk);

      final double rotationalRate = rotationalPidController.calculate(rotationalError, 0)
          * DriveTrainConstants.MaxAngularRate
          * 0.2;
      final double velocityX = xPidController.calculate(distToRobot, 2.25) * -1.0
          * DriveTrainConstants.MaxSpeed
          * 0.5;
      // final double velocityY = yPidController.calculate(fiducial.tync, 0) *
      // TunerConstants.MaxSpeed * 0.3;

      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint()) {
        this.end(true);
      }

      SmartDashboard.putNumber("FollowAprilTag/txnc", fiducial.tx_nocrosshair);
      SmartDashboard.putNumber("FollowAprilTag/distToRobot", distToRobot);
      SmartDashboard.putNumber("FollowAprilTag/rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("FollowAprilTag/xPidController", velocityX);
      drivetrain.setControl(
          alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));
      // .withVelocityY(velocityY));
    } catch (LimelightSubsystem.NoSuchTargetException nste) {
      SmartDashboard.putNumber("FollowAprilTag/TEST", -1);
    }
  }

  @Override
  public boolean isFinished() {
    boolean temp = rotationalPidController.atSetpoint() && xPidController.atSetpoint();
    if ( temp ) {
      System.out.println("AlignToApriltag_COMMAND IsFinished!");
    }
    return temp;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("FollowAprilTag_COMMAND ENDED!");
    drivetrain.applyRequest(() -> idleRequest);
  }
}