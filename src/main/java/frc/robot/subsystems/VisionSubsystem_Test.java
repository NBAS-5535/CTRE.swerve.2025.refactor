// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.VisionConstants;
import frc.robot.Vision.LimelightHelpers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/*
 * general trigonometry:
 * 
 */
public class VisionSubsystem_Test extends SubsystemBase {
    /** Creates a new VisionSubsystem. */
    private static NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");

    double targetOffsetAngle_Vertical;

    // limelight lens angle of the floor plane, non-zero in degrees if lifted upwards
    double limelightMountAngleDegrees = 0.0; 

    /* check Constants.java
    // height of the center of the Limelight lens from the floor (inches)
    double limelightLensHeightInches = 20.0; 

    // height of the target from the floor (inches)
    double targetHeightInches = 60.0; 
    */

    double angleToTargetDegrees;
    double angleToTargetRadians;

    //calculate distance to the target
    double distanceFromLimelightTargetInches;


    public VisionSubsystem_Test() {
        // set the correct mount angle
        //computeLimelightMountAngleDegrees(VisionConstants.knownDistance);
    }

    public boolean hasTarget() {
        return m_table.getEntry("tv").getDouble(0) == 1;
    }

    public double getTx() {
        return m_table.getEntry("tx").getDouble(0);
    }

    public double getTy() {
        return m_table.getEntry("ty").getDouble(0);
    }

    public double getTa() {
        return m_table.getEntry("ta").getDouble(0);
    }

    /*
     * Determine actual mount angle by a known distance - probably experimental
     */
    public void computeLimelightMountAngleDegrees(double distance){
        double ty = getTy();
        double limelightDegrees = Math.tan((VisionConstants.targetHeightInches - VisionConstants.limelightLensHeightInches) / distance) * 180 / Math.PI;
        limelightMountAngleDegrees = limelightDegrees - ty;
    }

    /*
     * compute the current distance to target (estimate)
     */
    public double getDistanceToTarget() {
        //Rotation2d angleToTarget = Rotation2d.fromDegrees(limelightMountAngleDegrees).plus(Rotation2d.fromDegrees(angleToTargetDegrees)))
        targetOffsetAngle_Vertical = getTy();
        angleToTargetDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        angleToTargetRadians = angleToTargetDegrees * (Math.PI / 180.0);
        distanceFromLimelightTargetInches = (VisionConstants.targetHeightInches - VisionConstants.limelightLensHeightInches) / Math.tan(angleToTargetRadians);
        SmartDashboard.putNumber("DistanceToTarget", distanceFromLimelightTargetInches);
        return distanceFromLimelightTargetInches;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight X Offset", getTx());
        SmartDashboard.putNumber("Limelight Y Offset", getTy());
        SmartDashboard.putNumber("Limelight Area", getTa());
    }

    public double getDistance() {
        Rotation2d angleToGoal = Rotation2d.fromDegrees(limelightMountAngleDegrees)
        .plus(Rotation2d.fromDegrees(LimelightHelpers.getTX("limelight")));
        double distance = (VisionConstants.targetHeightInches - VisionConstants.limelightLensHeightInches) / angleToGoal.getTan();
        return distance;
    }
}
