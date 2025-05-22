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
import frc.robot.Vision.LimelightHelpers.RawFiducial;

public class FollowAprilTag extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(
      2.5, 0, 0, 0.1);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.7, 0, 0, 0.02);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.7, 0, 0, 0.02);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  // private static final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();

  public FollowAprilTag(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    // addRequirements(this.drivetrain, this.limelight);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    LimelightTarget_Fiducial fiducial;
    SmartDashboard.putNumber("TEST", 0);
    try {
      fiducial = limelight.getTargetFiducialWithId(1);
      Pose3d targetPoseRobotSpace = fiducial.getTargetPose_RobotSpace();
      double distToRobot = targetPoseRobotSpace.getZ();
      double rotationalError = targetPoseRobotSpace.getRotation().getY();
      // double idk = fiducial.getTargetPose_RobotSpace().getY();
      SmartDashboard.putNumber("TEST", distToRobot);
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

      SmartDashboard.putNumber("txnc", fiducial.tx_nocrosshair);
      SmartDashboard.putNumber("distToRobot", distToRobot);
      SmartDashboard.putNumber("rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("xPidController", velocityX);
      drivetrain.setControl(
          alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));
      // .withVelocityY(velocityY));
    } catch (LimelightSubsystem.NoSuchTargetException nste) {
      SmartDashboard.putNumber("TEST", -1);
    }
  }

  @Override
  public boolean isFinished() {
    // return rotationalPidController.atSetpoint() && xPidController.atSetpoint()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> idleRequest);
  }
}