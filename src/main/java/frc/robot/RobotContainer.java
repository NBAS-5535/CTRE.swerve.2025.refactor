// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ActuatorSubsystemConstants;
import frc.robot.Constants.AutonomousMenuConstants;
import frc.robot.Constants.AutonomousModeOptions;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AlignToApriltagCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.FindAprilTagCommand;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.commands.ManualCommands;
import frc.robot.commands.OperatorFriendlyCommands;
import frc.robot.commands.SemiAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ActuatorSubsystem.ActuatorSetpoints;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.Setpoint;
import frc.robot.subsystems.Pigeon2GyroSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class RobotContainer {
    private double MaxSpeed = 0.8 * (TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final double speedDeadBand = 0.08;
    private final double angleDeadBand = 0.1;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * speedDeadBand).withRotationalDeadband(MaxAngularRate * angleDeadBand) // Add a deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.SysIdSwerveTranslation goForward = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //private final SwerveRequest.Fieldcentric
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController txbox = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* gyro */
    public final Pigeon2 pigeon2 = drivetrain.getPigeon2();
    public final Pigeon2GyroSubsystem pigeon2Subsystem = new Pigeon2GyroSubsystem(pigeon2);

    /* Limelight */
    /* initial attempt */
    private final LimelightSubsystem limelight = new LimelightSubsystem();
    private final VisionSubsystem m_vision = new VisionSubsystem();

    /* 2025 game related subsystems */
    /* actuator to move the levator to game start position */
    //public final boolean actuatorIsRev = false;
    public final ActuatorSubsystem m_actuator = new ActuatorSubsystem();

    /* elevator subsystem */
    //public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

    /** OurAlgaeSubsystem */
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

    /** LiftSubsystem */
    private final LiftSubsystem m_liftSubsystem = new LiftSubsystem();

        /* autonomous dropdown menu */
    private SendableChooser<String> dropDownChooser;// = new SendableChooser<>();
    private SendableChooser<String> autonomousChooser;
    
    /* scenario type menu */
    private SendableChooser<String> scenarioChooser;

    /* Limelight vision Apriltag chooser */
    private SendableChooser<Integer> aprilTagChooser;

    /* submenu */
    //private final SendableChooser<SendableChooser<String>> mainMenuChooser;
    int joystickDirection = 1;

    // flag to toggle PP
    private boolean joystickWithPP = false;

    /************** Ctor */
    public RobotContainer() {
        // create some NamedCommands for PathPlanner
        configureNamedCommands();
        //autoChooser = AutoBuilder.buildAutoChooser("TestPath");
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("PathPlanner Scenario", autoChooser);

        /* dropdown autonomous menu */
        dropDownChooser = new SendableChooser<>();
        dropDownChooser.setDefaultOption("Corral-Only", AutonomousModeOptions.kCorralOnly);
        dropDownChooser.addOption("Corral-Plus-Algae", AutonomousModeOptions.kCorralPlusAlgae);
        SmartDashboard.putData("Autonomous Mode Menu", dropDownChooser);
        

        /* autonomous position chooser */
        autonomousChooser = new SendableChooser<>();
        autonomousChooser.setDefaultOption("No Action", "None");
        autonomousChooser.addOption("Blue_1/Red-Barge-Side/Right", AutonomousMenuConstants.kDownBlue);
        autonomousChooser.addOption("Blue_2/Center", AutonomousMenuConstants.kCenterBlue);
        autonomousChooser.addOption("Blue_3/Blue-Barge-Side/Left", AutonomousMenuConstants.kUpBlue);
        autonomousChooser.addOption("Red_4/Red-Barge-Side/Left", AutonomousMenuConstants.kDownRed);
        autonomousChooser.addOption("Red_5/Center", AutonomousMenuConstants.kCenterRed);
        autonomousChooser.addOption("Red_6/Blue-Barge-Side/Right", AutonomousMenuConstants.kUpRed);
        SmartDashboard.putData("AutonomousMenu", autonomousChooser);

        scenarioChooser = new SendableChooser<String>();
        //scenarioChooser.setDefaultOption("Must Choose One", "None");
        scenarioChooser.setDefaultOption("Manually-Generated", "manual");
        scenarioChooser.addOption("PathPlanner", "path");
        SmartDashboard.putData("Scenario Type", scenarioChooser);

        /* AprilTag for vision chooser */
        aprilTagChooser = new SendableChooser<>();
        aprilTagChooser.setDefaultOption("No Pick", 0);
        for ( int i = 1 ; i <= VisionConstants.numberOfTags; i++) {
            aprilTagChooser.addOption("TagID: " + String.valueOf(i), i);
        }
        SmartDashboard.putData("AprilTag Options", aprilTagChooser);

        /*
        mainMenuChooser = new SendableChooser<SendableChooser<String>>();
        mainMenuChooser.setDefaultOption("Manual", autonomousChooser);
        mainMenuChooser.addOption("PathPlanner", autoChooser);
        */
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        /* alliance signal not available in simulation mode????? */
        boolean isCompetitionMode = true;
        joystickDirection = 1;

        joystickWithPP = true;
        
        if ( isCompetitionMode ) {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.get() == Alliance.Blue ) {
                if ( joystickWithPP ) {
                    joystickDirection = -1;
                } else {
                    joystickDirection = 1;
                }
            } else if (ally.get() == Alliance.Red ){
                //joystickDirection = -1;
                if ( joystickWithPP ) {
                    joystickDirection = 1;
                } else {
                    joystickDirection = -1;
                }
            } else {
                System.out.println(joystickDirection);
            }
        }
         
        System.out.println(joystickDirection);
        SmartDashboard.putBoolean("my direction", joystickWithPP);
        

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystickDirection * joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystickDirection * joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        /* This makes the current orientation of the robot X forward for field-centric maneuvers.  */
        // reset the field-centric heading on left bumper press
        //joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /* */
        //joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        /* Swerve DriveTrain */

        /* reused down below - check ourcoral
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        */ 
        
        // drive at a constant speed
        //joystick.y().whileTrue(drivetrain.applyRequest(() -> goForward.withVolts(0.2 * MaxSpeed)));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        boolean pidTest = true;
        if (pidTest) {
            double timeOut = 1.;
            joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));//.withTimeout(timeOut));
            joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));//.withTimeout(timeOut));
            joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));//.withTimeout(timeOut));
            joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));//.withTimeout(timeOut));
        } // end driveTest

        // prefixed movement in +/- X-direction
        /*
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        */

        // let's try rotation
        //joystick.povUp().onTrue(drivetrain.sysIdRotate(Direction.kForward));
        //joystick.povUp().whileTrue(drivetrain.sysIdRotate(Direction.kForward));

        // Rotate by 90deg using a fixed speed and time
        boolean rotationTest = false;
        if ( rotationTest ) {
            joystick.back().and(joystick.y()).onTrue(drivetrain.sysIdRotate(Direction.kForward).withTimeout(0.64));
            joystick.back().and(joystick.x()).onTrue(drivetrain.sysIdRotate(Direction.kReverse).withTimeout(0.63));
            joystick.start().and(joystick.y()).onTrue(drivetrain.sysIdRotate(Direction.kForward).withTimeout(0.62));
            joystick.start().and(joystick.x()).onTrue(drivetrain.sysIdRotate(Direction.kReverse).withTimeout(0.61));
            //joystick.x().whileTrue(drivetrain.sysIdRotate(Direction.kForward));
            //joystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
         
            //drivetrain.getModule(0).getDriveMotor().getPosition().getValue();
        } // end driveTest

        // point forward
        /*
        joystick.povRight().onTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(tempAngle))
        ));
        */
        /* Rotate by 90deg in-place using a fixed rotational speed
        joystick.povLeft().whileTrue(drivetrain.applyRequest(() ->
            drive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(tempAngle * 0.1 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
        */
        // Rotate by a specific angle
        double tempAngle = Math.PI / 2.;
        SmartDashboard.putNumber("Angle", tempAngle);
        SmartDashboard.putNumber("MaxAngularVelocity", MaxAngularRate);

        /* rotate by setting a marker and turning the robot until 90deg is reached */
        boolean pigeonTest = false;
        if (pigeonTest) {
            //joystick.povDown().whileTrue(new OperatorFriendlyCommands(drivetrain, pigeon2Subsystem));
            //pigeon2Subsystem.setAngleMarker();
            //SmartDashboard.putNumber("Reference Angle", pigeon2Subsystem.getHeading());
            /* rotate robot "gradually" until ~90deg is reached*/
            joystick.b().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> pigeon2Subsystem.setAngleMarker()),
                drivetrain.sysIdRotate(Direction.kForward).until(() -> pigeon2Subsystem.isAngleDiffReached(drivetrain, 90.)))
                );
            
        } // end driveTest

        boolean multiDimensionalMove = false;
        if ( multiDimensionalMove ) {
            final double speed = 1.0; //m/s
            final double angularSpeed = Math.PI / 4.5;
            SmartDashboard.putNumber("Test Speed", speed);
            joystick.b().onTrue(drivetrain.applyRequest(() -> drivetrain.robotCentricMove.withVelocityX(speed)
                                                                                        .withVelocityY(-0.)
                                                                                        .withRotationalRate(-angularSpeed)
                                                        //).withTimeout(0.5));
                                                        ).until(() -> drivetrain.isDesiredPoseReached(Math.abs(1.8))));
        }
        /* get robot Pose/location info */
        boolean poseTest = false;
        if ( poseTest ) {
            /* pedantic way */
            /*
            joystick.povRight().onTrue(new SequentialCommandGroup(
                new OperatorFriendlyCommands(drivetrain, pigeon2Subsystem, "pose"),
                drivetrain.sysIdDynamic(Direction.kForward).withTimeout(1),
                new OperatorFriendlyCommands(drivetrain, pigeon2Subsystem, "pose")
            ));
            */
            /* fancy way: 
            * Mark current position
            * Drive forward for 2 meters
            */
            txbox.povDown().onTrue(new SequentialCommandGroup(
                //new InstantCommandMarkGyroPose(drivetrain),
                new InstantCommand(() -> drivetrain.setCurrentPose()),
                drivetrain.sysIdDynamic(Direction.kForward).until(() -> drivetrain.isDesiredPoseReached(1.5))
            ));
            
        } // end driveTest

        /* Vision Subsystem */
        boolean visionTest = true;
        if (visionTest) {
            /* pick an Apriltag ID from the menu */
            //int aprilTagID = aprilTagChooser.getSelected();
            int aprilTagID = 5;

            // get vision-based distance
            //joystick.x().onTrue(new InstantCommand(() -> m_vision.getDistanceToTarget()));
            /* onTrue: robot moves until the alignment is completed
            *  whileTrue: must press the button until the alignment is completed
            */
            //joystick.x().onTrue(new AlignCommand(drivetrain, m_vision, 5));
            joystick.x().onTrue(new FindAprilTagCommand(drivetrain, m_vision, 5));
            /* simulate a sequence:
            * align with AprilTag
            */
            /* Testing Closest versus a specific AprilTag */
            boolean closestTag = true;
            // Default: a specific tag number
            int testTagId = VisionConstants.testTagId;
            if ( closestTag) {
                testTagId = 0;
            }
            
            joystick.y().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.setCurrentPose()),
                new AlignCommand(drivetrain, m_vision, testTagId),
                //drivetrain.applyRequest(() -> brake),
                drivetrain.sysIdDynamic(Direction.kForward).withTimeout(1.)//.until(() -> drivetrain.isDesiredPoseReached(1.))
            ));
            
            joystick.povUp().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.setCurrentPose()),
                new AlignCommand(drivetrain, m_vision, 1),
                //drivetrain.applyRequest(() -> brake),
                drivetrain.sysIdDynamic(Direction.kForward).until(() -> drivetrain.isDesiredPoseReached(1.))
            ));

            joystick.povDown().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.setCurrentPose()),
                new AlignCommand(drivetrain, m_vision, 5),
                //drivetrain.applyRequest(() -> brake),
                drivetrain.sysIdDynamic(Direction.kForward).until(() -> drivetrain.isDesiredPoseReached(2.))
            ));

            /* available joystick slots for further testing
             * povUp()
             * povDown()
            */
            /* get mindistance info 
            joystick.povUp().whileTrue(
                new AlignToApriltagCommand(drivetrain, limelight, aprilTagID)
            );

            joystick.povDown().whileTrue(
                new FollowAprilTagCommand(drivetrain, limelight, aprilTagID)
            );
            */

        } // end visionTest

        /* Actuator Subsystem */
        boolean actuatorTest = true;
        if (actuatorTest) {
            
        // move the elevator to game position: direction =1
            /* uncomment if needed */
            joystick.povRight().onTrue(
                m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
            );
            // move elevator back to start position
            joystick.povLeft().onTrue(
                m_actuator.setSetpointCommand(ActuatorSetpoints.kBase)
            );
            // move to an intermediate point
            //joystick.start().onTrue(
            //    m_actuator.setSetpointCommand(ActuatorSetpoints.kAlgaeNetShootSetPoint)
            //);
            /**/
            
        } // end actuator test buttons

        /* Algae Subsystem */
        boolean algaeSubsystemTuning = false;
        if (algaeSubsystemTuning) {
            /* Try gradually moving the elevator to determine operational heights */
            // A -> Run elevator UP

            joystick.a().whileTrue(ManualCommands.runElevatorUpCommand(m_algaeSubsystem));
            // B -> Run elevator DOWN
            joystick.b().whileTrue(ManualCommands.runElevatorDownCommand(m_algaeSubsystem));

            /* Try gradually moving the arm to determine operational heights */
            // povUp -> Run arm UP
            joystick.povUp().whileTrue(ManualCommands.runArmUpCommand(m_algaeSubsystem));
            // povDown -> Run arm DOWN
            joystick.povDown().whileTrue(ManualCommands.runArmDownCommand(m_algaeSubsystem));
        } // end of elevator/arm test buttons

        /* quick test code 

            // A -> Run elevator UP
            joystick.a().whileTrue(ManualCommands.runElevatorUpCommand(m_algaeSubsystem));
            // B -> Run elevator DOWN
            joystick.b().whileTrue(ManualCommands.runElevatorDownCommand(m_algaeSubsystem));
            
            // start -> Run arm UP
            joystick.a().whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> m_algaeSubsystem.setPeriodicToFalse()),
                ManualCommands.runArmUpCommand(m_algaeSubsystem),
                new InstantCommand(() -> m_algaeSubsystem.resetPeriodicMode())
                )
            );
            // back -> Run arm DOWN
            //joystick.back().whileTrue(ManualCommands.runArmDownCommand(m_algaeSubsystem));
            joystick.b().whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> m_algaeSubsystem.setPeriodicToFalse()),
                ManualCommands.runArmDownCommand(m_algaeSubsystem),
                new InstantCommand(() -> m_algaeSubsystem.resetPeriodicMode())
                )
            );
            */
        
        
        /* run Algae intake motor in suck-in and push-out modes */
        // rightBumper() -> Run tube intake
        joystick.rightBumper().whileTrue(ManualCommands.runIntakeCommand(m_algaeSubsystem));

        // a() -> Run tube intake
        joystick.a().whileTrue(ManualCommands.reverseIntakeCommandSlow(m_algaeSubsystem));

        // leftBumper() -> Run tube intake in reverse
        joystick.leftBumper().whileTrue(ManualCommands.reverseIntakeCommand(m_algaeSubsystem));


        /* run lift motor in suck-in and push-out modes */
        boolean useLiftSubsystem = false;
        if ( useLiftSubsystem ) {
            // y() -> Lifting the robot UP
            joystick.y().whileTrue(m_liftSubsystem.runLiftUpCommand());

            // start() -> Lowering the robot DOWN
            joystick.start().whileTrue(m_liftSubsystem.runLiftDownCommand());
        }
        /**/
        
        // move elevator/arm to their respective positions
        txbox
            .a()
                .onTrue(
                    /*
                    //m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupLowerReef)
                    //new SequentialCommandGroup(
                            m_algaeSubsystem.setSetpointCommand(Setpoint.kClearWires)//,
                                .andThen(m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                                .andThen(m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupLowerReef))
                    */
                    m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupLowerReef)
                        .alongWith(m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions))             
        );

        txbox
            .b()
                .onTrue(
                    m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupHigherReef)
                        .alongWith(m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions))
                    /*
                    new SequentialCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kClearWires)//,
                            .andThen(m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions))
                            .andThen(m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupHigherReef))//,
                        //Commands.waitSeconds(1.),
                        //m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                    )
                    //m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupHigherReef)
                    */                   
        );
        
        txbox
            .x()
                .onTrue(
                    //m_algaeSubsystem.setSetpointCommand(Setpoint.kGroundPickup)
                    /**/
                    new ParallelCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kGroundPickup),
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                    )
                       /* */
        );

        txbox
            .y()
                .onTrue(
                    //m_algaeSubsystem.setSetpointCommand(Setpoint.kShootAlgaeNet)
                    new ParallelCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kShootAlgaeNet),
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kAlgaeNetShootSetPoint)
                    )
        );

        //
        txbox
            .rightBumper()
                .onTrue(
                    new SequentialCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kMoveWithBall)//,
                        //Commands.waitSeconds(0.5),
                            .andThen( m_algaeSubsystem.setSetpointCommand(Setpoint.kSideSlotShoot)),
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                    )    
        );

        txbox
            .povUp()
                .onTrue(
                    new SequentialCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kSideSlotShoot),
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                    )    
        );

        /* corral drop: most likely an autonomous functionality */
        txbox
            .povDown()
                .onTrue(
                    new SequentialCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kCorralDrop),
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kAlgaeNetShootSetPoint),
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kShootCorralDrop)
                    )
        );

        /* move with ball */
        txbox
            .leftBumper()
                .onTrue(
                new SequentialCommandGroup(
                        m_algaeSubsystem.setSetpointCommand(Setpoint.kMoveWithBall),
                        //m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions)
                        m_actuator.setSetpointCommand(ActuatorSetpoints.kBase)
                    )
        );
            
      
    
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // associate PathPlanner NamedCommands with actual function calls
    private void configureNamedCommands() {
        NamedCommands.registerCommand("EjectAlgae", m_algaeSubsystem.reverseIntakeCommandSlow().withTimeout(0.5));
        NamedCommands.registerCommand("MoveCorralToLowerReefLevel", Autos.moveCorralToLowerReefLevel(m_algaeSubsystem, m_actuator));
        NamedCommands.registerCommand("DropCorralToLowerReefLevel", Autos.dropCorralToLowerReefLevel(m_algaeSubsystem, m_actuator));
        //NamedCommands.registerCommand("PickupAlgaeFromLowReef", Autos.pickupAlgaeFromLowReef(m_algaeSubsystem));
        //NamedCommands.registerCommand("PickupAlgaeFromHighReef", Autos.pickupAlgaeFromHighReef(m_algaeSubsystem));
        NamedCommands.registerCommand("StraightenActuatorEjectCorral", 
                                      new SequentialCommandGroup(m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions),
                                                                 Commands.waitSeconds(0.5),
                                                                 m_algaeSubsystem.runIntakeCommand().withTimeout(0.25)));
        NamedCommands.registerCommand("StraightenActuator", m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions));
        NamedCommands.registerCommand("ReadyForAlgaePickup", 
                                      new SequentialCommandGroup(//m_algaeSubsystem.setSetpointCommand(Setpoint.kClearWires),
                                                                 m_algaeSubsystem.setSetpointCommand(Setpoint.kClearReef),
                                                                 //Commands.waitSeconds(0.25),
                                                                 m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupLowerReef)));
        NamedCommands.registerCommand("IntakeAlgaeFromReef", m_algaeSubsystem.runIntakeCommand().withTimeout(0.25));
        NamedCommands.registerCommand("GoToHighReefPickupPosition", m_algaeSubsystem.setSetpointCommand(Setpoint.kAlgaePickupHigherReef));
        NamedCommands.registerCommand("RaiseForAlgaeNetShooting", 
                                      new SequentialCommandGroup(
                                                    m_actuator.setSetpointCommand(ActuatorSetpoints.kAlgaeNetShootSetPoint),
                                                    Commands.waitSeconds(0.5),
                                                    m_algaeSubsystem.setSetpointCommand(Setpoint.kShootAlgaeNet)
                                                ));
        NamedCommands.registerCommand("MoveAndDropCorralAtReef", 
                                        Autos.moveCorralToLowerReefLevel(m_algaeSubsystem, m_actuator)
                                            .andThen(m_actuator.setSetpointCommand(ActuatorSetpoints.kSetPointInRevolutions))
                                            .andThen(m_algaeSubsystem.runIntakeCommand().withTimeout(0.25)));
        
    }

    public Command getAutonomousCommand() {
        // some autonomous sequences
        String caseType = scenarioChooser.getSelected(); //"path"; //"manual";
        Command autoCommand = null;
        String menuItem = "";
        String chosenItem = "";
        String modeOption = AutonomousModeOptions.kCorralOnly;
        int direction = 1;
        double angle = 0.;
        SmartDashboard.putString("Auto Sequence", caseType);
        switch (caseType) {
            case "pedantic":
                autoCommand = Commands.sequence(
                    drivetrain.sysIdDynamic(Direction.kForward).withTimeout(0.5),
                    //drivetrain.applyRequest(() -> brake),
                    Commands.waitSeconds(3.0),
                    drivetrain.sysIdRotate(Direction.kForward).withTimeout(0.34),
                    Commands.waitSeconds(1.),
                    drivetrain.sysIdRotate(Direction.kForward).withTimeout(0.68),
                    Commands.waitSeconds(2.),
                    drivetrain.sysIdDynamic(Direction.kForward).withTimeout(0.5)
                );
                break;
            case "auto":
                autoCommand = Autos.moveRotateRestRepeat(drivetrain);
                break;
            case "manual":
                //autoCommand = dropDownChooser.getSelected();
                /* IMPORTANT: Designations are wrt PathPlanner layout: 
                *             Blue on the left / Red on the right 
                *             Blue AlgeNet/barge is BlueUP
                *             Red AlgeNet/barge is RedDOWN
                */
                menuItem = autonomousChooser.getSelected();
                modeOption = dropDownChooser.getSelected();
                chosenItem = "NO ACTION";
                angle = 57.;
                switch (menuItem){
                    case AutonomousMenuConstants.kDownBlue:
                        chosenItem = "Blue_1/Red-Barge-Side";
                        direction = -1;
                        switch (modeOption){
                            case AutonomousModeOptions.kCorralOnly:
                                autoCommand = Autos.algaenetSideStart_CorralOnly(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                //autoCommand = Autos.moveOffTheLine(drivetrain, Direction.kForward);
                            case AutonomousModeOptions.kCorralPlusAlgae:
                                autoCommand = Autos.algaenetSideStart(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                break;
                            default:
                                autoCommand = Autos.algaenetSideStart_CorralOnly(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                break;
                        }
                        break;
                    case AutonomousMenuConstants.kCenterBlue:
                        chosenItem = "Blue_2/Center";
                        switch (modeOption){
                            case AutonomousModeOptions.kCorralOnly:
                                autoCommand = Autos.midlineStart_scoreCorralOnly(drivetrain, pigeon2Subsystem, m_algaeSubsystem, m_actuator);
                                break;
                            case AutonomousModeOptions.kCorralPlusAlgae:
                                autoCommand = Autos.midlineStartCommand(drivetrain, pigeon2Subsystem, m_algaeSubsystem, m_actuator);
                                //SmartDashboard.putString("WhatsUP?", modeOption);
                                break;
                            default:
                                autoCommand = Autos.midlineStart_scoreCorralOnly(drivetrain, pigeon2Subsystem, m_algaeSubsystem, m_actuator);
                                break;
                        }
                        break;
                    case AutonomousMenuConstants.kUpBlue:
                        chosenItem = "Blue_3/Blue-Barge-Side";
                        direction = 1;
                        switch (modeOption){
                            case AutonomousModeOptions.kCorralOnly:
                                autoCommand = Autos.algaenetSideStart_CorralOnly(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                //autoCommand = Autos.moveOffTheLine(drivetrain, Direction.kForward);
                            case AutonomousModeOptions.kCorralPlusAlgae:
                                autoCommand = Autos.algaenetSideStart(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                break;
                            default:
                                autoCommand = Autos.algaenetSideStart_CorralOnly(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                break;
                        }
                        break;
                    case AutonomousMenuConstants.kDownRed:
                        chosenItem = "Red_4/Red-Barge-Side";
                        direction = 1;
                        switch (modeOption){
                            case AutonomousModeOptions.kCorralOnly:
                                autoCommand = Autos.algaenetSideStart_CorralOnly(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                //autoCommand = Autos.moveOffTheLine(drivetrain, Direction.kForward);
                                break;
                            case AutonomousModeOptions.kCorralPlusAlgae:
                                autoCommand = Autos.algaenetSideStart(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                break;
                            default:
                                autoCommand = Autos.algaenetSideStart_CorralOnly(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                break;
                        }
                        break;
                    case AutonomousMenuConstants.kCenterRed:
                        chosenItem = "Red_5/Center";
                        switch (modeOption){
                            case AutonomousModeOptions.kCorralOnly:
                                autoCommand = Autos.midlineStart_scoreCorralOnly(drivetrain, pigeon2Subsystem, m_algaeSubsystem, m_actuator);
                                break;
                            case AutonomousModeOptions.kCorralPlusAlgae:
                                autoCommand = Autos.midlineStartCommand(drivetrain, pigeon2Subsystem, m_algaeSubsystem, m_actuator);
                                //SmartDashboard.putString("CorralStuff", "With Algae Throw");
                                break;
                            default:
                                autoCommand = Autos.midlineStart_scoreCorralOnly(drivetrain, pigeon2Subsystem, m_algaeSubsystem, m_actuator);
                                //SmartDashboard.putString("CorralStuff", "Hit default");
                                break;
                        }
                        break;
                    case AutonomousMenuConstants.kUpRed:
                        chosenItem = "Red_6/Blue-Barge-Side";
                        direction = -1;
                        switch (modeOption){
                            case AutonomousModeOptions.kCorralOnly:
                                autoCommand = Autos.algaenetSideStart_CorralOnly(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                //autoCommand = Autos.moveOffTheLine(drivetrain, Direction.kForward);
                                break;
                            case AutonomousModeOptions.kCorralPlusAlgae:
                                autoCommand = Autos.algaenetSideStart(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                break;
                            default:
                                autoCommand = Autos.algaenetSideStart_CorralOnly(drivetrain, pigeon2Subsystem, 
                                                                                 m_algaeSubsystem, m_actuator, 
                                                                                 direction, angle);
                                break;
                        }
                        break; 
                    default:
                        chosenItem = "Nothing"; 
                }
                SmartDashboard.putString("Menu-Pick", chosenItem);
                break;
            case "path":
                /* Run the path selected from the auto chooser */
                //autoCommand = new PathPlannerAuto("FancyAutoPath"); //
                autoCommand = autoChooser.getSelected();
                //joystickWithPP = true;

                /*
                try{
                    // Load the path you want to follow using its name in the GUI
                    PathPlannerPath path = PathPlannerPath.fromPathFile("AutoFancyCurve");

                    // Create a path following command using AutoBuilder. This will also trigger event markers.
                    autoCommand = AutoBuilder.followPath(path);
                } catch (Exception e) {
                    DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                    autoCommand =  Commands.none();
                }
                */
                break;
            default:
                autoCommand = Commands.print("No autonomous command configured");
        }
        return autoCommand;

    }

}
