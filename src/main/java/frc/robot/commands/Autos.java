// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Pigeon2GyroSubsystem;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ActuatorSubsystem.ActuatorSetpoints;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.Setpoint;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autos extends Command {
  /** Creates a new Autos. */
  static Timer timer;
  public Autos() {
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /* test */
  public static Command moveRotateRestRepeat(CommandSwerveDrivetrain swerve){
    Command autoCommand = Commands.sequence(
      swerve.sysIdDynamic(Direction.kForward).withTimeout(0.1),
            Commands.waitSeconds(5.0),
            swerve.sysIdRotate(Direction.kForward).withTimeout(0.65), // for 90deg, rotate for 0.333s at pi rad/s
            Commands.waitSeconds(5.),
            swerve.sysIdRotate(Direction.kForward).withTimeout(0.65),
            Commands.waitSeconds(5.),
            swerve.sysIdDynamic(Direction.kReverse).withTimeout(0.1)
        );
    return autoCommand;
  }

  /* Game scenarios */
  /* move off the start line by driving forward for 1 sec */
  public static Command moveOffTheLine(CommandSwerveDrivetrain swerve, Direction direction){
    double timeToTravel = 4.; //sec at 1m/s 
    return swerve.sysIdDynamic(direction).withTimeout(timeToTravel);
  }

  /* move from line to reef; 
   * move actuator straight;
   * drop corral;
   * move to low reef seeting
   * pick up algae
   * move back
   * rotate 90deg
   * move to barge net
   * rotate 90deg
   * move towarda barge
   * move algae shoot setting
   * eject algae
   */
  public static Command midlineStartCommandPedantic(CommandSwerveDrivetrain swerve, 
                                            Pigeon2GyroSubsystem gyro, 
                                            AlgaeSubsystem algae,
                                            ActuatorSubsystem actuator) {
    Command tempCommand;
    tempCommand = new SequentialCommandGroup(
      //move forward
      new SequentialCommandGroup(
                //new InstantCommandMarkGyroPose(drivetrain),
                new InstantCommand(() -> swerve.setCurrentPose()),
                swerve.sysIdDynamic(Direction.kForward).until(() -> swerve.isDesiredPoseReached(2.1))
                ),
      // add corraldrop and low reef pickup
      new SequentialCommandGroup(
                algae.setSetpointCommand(Setpoint.kCorralDrop),
                actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions),
                algae.setSetpointCommand(AlgaeSubsystem.Setpoint.kCorralDrop)                
                ),
      ManualCommands.runIntakeCommand(algae).withTimeout(1.5),
      // move back
      new SequentialCommandGroup(
                  //new InstantCommandMarkGyroPose(drivetrain),
                  new InstantCommand(() -> swerve.setCurrentPose()),
                  swerve.sysIdDynamic(Direction.kForward).until(() -> swerve.isDesiredPoseReached(0.6))
                  ),
      // rotate set angel tol to 10deg??
      new SequentialCommandGroup(
                new InstantCommand(() -> gyro.setAngleMarker()),
                swerve.sysIdRotate(Direction.kForward).until(() -> gyro.isAngleDiffReached(swerve, 90.))
                ),
      //move to wards barge
      new SequentialCommandGroup(
                //new InstantCommandMarkGyroPose(drivetrain),
                new InstantCommand(() -> swerve.setCurrentPose()),
                swerve.sysIdDynamic(Direction.kForward).until(() -> swerve.isDesiredPoseReached(2.0))
                ),
      // rotate
      new SequentialCommandGroup(
                new InstantCommand(() -> gyro.setAngleMarker()),
                swerve.sysIdRotate(Direction.kForward).until(() -> gyro.isAngleDiffReached(swerve, 90))
                ),
      new SequentialCommandGroup(
                //new InstantCommandMarkGyroPose(drivetrain),
                new InstantCommand(() -> swerve.setCurrentPose()),
                swerve.sysIdDynamic(Direction.kForward).until(() -> swerve.isDesiredPoseReached(0.8))
                )
      // add highreef setting and shooting

    );
    return tempCommand;
  }

  /* action to be stitched together if desired */
  /* go distance: in encoder value */
  public static Command moveByDistance(CommandSwerveDrivetrain swerve, double encoderPosition) {
    Direction direction = Direction.kForward;
    if ( encoderPosition < 0. ) {
      direction = Direction.kReverse;
    }
    return new SequentialCommandGroup(
      //new InstantCommandMarkGyroPose(drivetrain),
      new InstantCommand(() -> swerve.setCurrentPose()),
      swerve.sysIdDynamic(direction).until(() -> swerve.isDesiredPoseReached(Math.abs(encoderPosition)))
      );
  }

    /* go distance: in encoder value */
    public static Command moveByDistanceInXY(CommandSwerveDrivetrain swerve, double encoderPosition) {

      final double speedX = Math.signum(encoderPosition) * (1.); //m/s
      final double speedY = 0.;
      final double angularSpeed = Math.PI / 4.1; // need a bit faster?

      return swerve.applyRequest(() -> swerve.robotCentricMove.withVelocityX(speedX)
                                                             .withVelocityY(speedY)
                                                             .withRotationalRate(-angularSpeed))
                                    .withTimeout(0.1);
                                 //.until(() -> swerve.isDesiredPoseReached(Math.abs(encoderPosition)));
    }

  /* experimental
  public static Command moveByDistanceInXY(CommandSwerveDrivetrain swerve, double encoderPosition, double joystickDirection) {
    Direction direction = Direction.kForward;
    if ( encoderPosition < 0. ) {
      direction = Direction.kReverse;
    }
    return new SequentialCommandGroup(
      //new InstantCommandMarkGyroPose(drivetrain),
      new InstantCommand(() -> swerve.setCurrentPose()),
      drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(joystickDirection * speed, joystickDirection * speed)).until(() -> swerve.isDesiredPoseReached(Math.abs(encoderPosition)))
      );
  }
      */

  /* rotate by angle using Pigeon info */
  public static Command rotateByAngleInDegrees(CommandSwerveDrivetrain swerve, 
                                               Pigeon2GyroSubsystem gyro,
                                               double angle) {
    Direction direction = Direction.kForward;
    if (angle < 0. ) {
      direction = Direction.kReverse;
    }
    return new SequentialCommandGroup(
      new InstantCommand(() -> gyro.setAngleMarker()),
      swerve.sysIdRotate(direction).until(() -> gyro.isAngleDiffReached(swerve, Math.abs(angle)))
      );
  }

  /* rotate by angle using Pigeon info */
  public static Command rotateByTime(CommandSwerveDrivetrain swerve, 
                                     Direction direction) {
    return swerve.sysIdRotate(direction);
  }

  /* position for corral drop */
  public static Command moveCorralToLowerReefLevel(AlgaeSubsystem algae,
                                               ActuatorSubsystem actuator) {
    return new SequentialCommandGroup(
      algae.setSetpointCommand(Setpoint.kCorralDrop),
      actuator.setSetpointCommand(ActuatorSetpoints.kAlgaeNetShootSetPoint),
      algae.setSetpointCommand(Setpoint.kShootCorralDrop)//,
      //algae.runIntakeCommand()//.withTimeout(1.) // to eject the corral
      );
  }

  /* position for corral drop */
  public static Command dropCorralToLowerReefLevel(AlgaeSubsystem algae,
  ActuatorSubsystem actuator) {
    return new SequentialCommandGroup(
      actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions), //straighten actuator
      Commands.waitSeconds(0.5),
      algae.runIntakeCommand().withTimeout(0.5) // to eject the corral
    );
  }

  /* activate intake and move to low algae pickup */
  public static Command pickupAlgaeFromLowReef(AlgaeSubsystem algae) {
    return new SequentialCommandGroup(
      algae.setSetpointCommand(Setpoint.kAlgaePickupLowerReef),
      algae.runIntakeCommand().withTimeout(1.) // retrieve algae
    );
  }

  /* activate intake and move to low algae pickup */
  public static Command pickupAlgaeFromHighReef(AlgaeSubsystem algae) {
    return new SequentialCommandGroup(
      algae.setSetpointCommand(Setpoint.kAlgaePickupHigherReef),
      algae.runIntakeCommand().withTimeout(1.) // retrieve algae
    );
  }

  /* move to algae shoot and activate intake in reverse */
  public static Command shootAlgaeIntoNet(AlgaeSubsystem algae) {
    return new SequentialCommandGroup(
      algae.setSetpointCommand(Setpoint.kShootAlgaeNet),
      Commands.waitSeconds(2.),
      algae.reverseIntakeCommand().withTimeout(1.)
    );
  }

  /* COMPOSITE Commands */
  /* take a corral to the reef */
  public static Command midlineStart_scoreCorralOnly(CommandSwerveDrivetrain swerve, 
                                                Pigeon2GyroSubsystem gyro, 
                                                AlgaeSubsystem algae,
                                                ActuatorSubsystem actuator) {
    double timeout = 2.; // seconds between commands
    Command tempCommand = new SequentialCommandGroup(
        moveByDistance(swerve, 1.5),            //move forward 88"

        moveCorralToLowerReefLevel(algae, actuator),                //drop corral
        actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions), //straighten actuator
        Commands.waitSeconds(timeout),
        algae.runIntakeCommand().withTimeout(0.5), // to eject the corral
        Commands.waitSeconds(timeout),

        // get ready for TeleOp action
        algae.setSetpointCommand(Setpoint.kClearWires),
        algae.setSetpointCommand(Setpoint.kClearReef),
        Commands.waitSeconds(0.5),
        algae.setSetpointCommand(Setpoint.kAlgaePickupLowerReef)
    );

    return tempCommand;
  }

  /* midline start commmand reimagined */
  public static Command midlineStartCommand(CommandSwerveDrivetrain swerve, 
                                    Pigeon2GyroSubsystem gyro, 
                                    AlgaeSubsystem algae,
                                    ActuatorSubsystem actuator) {
    double timeout = 0.5; // seconds between commands
    
    System.out.println(Timer.getFPGATimestamp());
    Command tempCommand = new SequentialCommandGroup(
      moveByDistance(swerve, 1.05),            //move forward 88" or 0.95 if robot end on the line
      Commands.waitSeconds(timeout),
      //Commands.waitSeconds(timeout),
      /**/
      moveCorralToLowerReefLevel(algae, actuator),                //drop corral
      actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions), //straighten actuator
      Commands.waitSeconds(1.0),
      algae.runIntakeCommand().withTimeout(0.5), // to eject the corral
      Commands.waitSeconds(0.5),

      // get ready for TeleOp action
      algae.setSetpointCommand(Setpoint.kClearWires),
      algae.setSetpointCommand(Setpoint.kClearReef),
      Commands.waitSeconds(0.5),
      algae.setSetpointCommand(Setpoint.kAlgaePickupLowerReef),
      /**/
      Commands.waitSeconds(timeout),
      
      moveByDistance(swerve, 0.3),            //move closer for pickup
      //Commands.waitSeconds(timeout),
      
      algae.runIntakeCommand().withTimeout(0.5), // to get algae
      //Commands.waitSeconds(timeout),
      
      moveByDistance(swerve, -0.6),            //move back to rotate
      
      
      rotateByAngleInDegrees(swerve, gyro, -90.),        //rotate 90deg
      //Commands.waitSeconds(timeout),
      
      moveByDistanceInXY(swerve, 1.)
      /*
      Commands.waitSeconds(0.75),
      algae.setSetpointCommand(Setpoint.kShootAlgaeNet),
      Commands.waitSeconds(2.),
      algae.reverseIntakeCommand().withTimeout(0.5),
      Commands.waitSeconds(1.),
      algae.setSetpointCommand(Setpoint.kAlgaePickupLowerReef),
      moveByDistance(swerve, -0.6)
       
      moveByDistance(swerve, 1.),            //move to algae net/barge ~100"
      //Commands.waitSeconds(timeout),
      
      rotateByAngleInDegrees(swerve, gyro, -90.),        //rotate 90deg towards algaenet
      //Commands.waitSeconds(timeout),
      
      moveByDistance(swerve, 0.9),            //move closer to algae net/barge
      Commands.waitSeconds(timeout),
      */
      //shootAlgaeIntoNet(algae),                                //shoot algae into net
      //algae.setSetpointCommand(Setpoint.kShootAlgaeNet),
      //Commands.waitSeconds(2.),
      //algae.reverseIntakeCommand().withTimeout(0.5),

      //Commands.waitSeconds(timeout),
      //algae.setSetpointCommand(Setpoint.kAlgaePickupLowerReef)
      /**/
    );
    System.out.println(timer.getFPGATimestamp());
    return tempCommand;
  }

  /********** SIDE START */
  /* start in front of the Blue or Red barge */
  public static Command algaenetSideStart_CorralOnly(CommandSwerveDrivetrain swerve, 
                                          Pigeon2GyroSubsystem gyro, 
                                          AlgaeSubsystem algae,
                                          ActuatorSubsystem actuator,
                                          int direction,
                                          double angle) {
    double timeout = 1.; // seconds between commands
    angle *= direction;
    Command tempCommand = new SequentialCommandGroup(
      moveByDistance(swerve, 1.7),            //move closer for pickup 
      //Commands.waitSeconds(timeout),

      rotateByAngleInDegrees(swerve, gyro, angle), //rotate toward reef side by ANGLE 55 instead of 60
      moveByDistance(swerve, 1.1),            //move forward 
      Commands.waitSeconds(timeout),
      
      moveCorralToLowerReefLevel(algae, actuator),                //drop corral
      actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions), //straighten actuator
      Commands.waitSeconds(timeout),
      algae.runIntakeCommand().withTimeout(0.5), // to eject the corral
      Commands.waitSeconds(timeout),

      // get ready for TeleOp action
      algae.setSetpointCommand(Setpoint.kClearWires),
      algae.setSetpointCommand(Setpoint.kAlgaePickupHigherReef)
    );
    return tempCommand;
  }

   /* start in front of the Blue or Red barge */
   public static Command algaenetSideStart(CommandSwerveDrivetrain swerve, 
                                          Pigeon2GyroSubsystem gyro, 
                                          AlgaeSubsystem algae,
                                          ActuatorSubsystem actuator,
                                          int direction,
                                          double angle) {
    double timeout = 2.; // seconds between commands
    angle *= direction;
    Command tempCommand = new SequentialCommandGroup(
    moveByDistance(swerve, 1.7),            //move closer for pickup 
    //Commands.waitSeconds(timeout),

    rotateByAngleInDegrees(swerve, gyro, angle), //rotate toward reef side by ANGLE 55 instead of 60
    moveByDistance(swerve, 1.2),            //move forward 
    Commands.waitSeconds(timeout),

    moveCorralToLowerReefLevel(algae, actuator),                //drop corral
    actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions), //straighten actuator
    Commands.waitSeconds(timeout),
    algae.runIntakeCommand().withTimeout(0.5), // to eject the corral
    Commands.waitSeconds(timeout),

    // get ready for TeleOp action
    algae.setSetpointCommand(Setpoint.kClearWires),
    algae.setSetpointCommand(Setpoint.kAlgaePickupHigherReef),
    Commands.waitSeconds(timeout),
      
    moveByDistance(swerve, 0.3),            //move closer for pickup
    //Commands.waitSeconds(timeout),
    
    algae.runIntakeCommand().withTimeout(0.5), // to get algae
    //Commands.waitSeconds(timeout),
    
    moveByDistance(swerve, -0.9),            //move back to rotate
    rotateByAngleInDegrees(swerve, gyro, angle/2.)
    );
    return tempCommand;
  }
}
