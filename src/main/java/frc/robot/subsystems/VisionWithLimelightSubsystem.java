// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

import frc.robot.Vision.LimelightHelpers;
import frc.robot.Vision.LimelightHelpers.RawFiducial;
import frc.robot.Vision.LimelightHelpers.LimelightResults;
import frc.robot.Vision.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionWithLimelightSubsystem extends SubsystemBase {
  private final String limelightName = "";
  private RawFiducial[] rawFiducials;

  /** Creates a new VisionWithLimelightSubsystem. */
  public VisionWithLimelightSubsystem() {
    config();
  }

  public void config() {

    // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);
    LimelightHelpers.setCameraPose_RobotSpace(
      this.limelightName,
        0.38, //meters Meters.convertFrom(30. / 2., Inches), // forward location wrt robot center
        0.25, // assume perfect alignment with robor center
        0.26, //m Meters.convertFrom(VisionConstants.limelightLensHeightInches, Inches), // height of camera from the floor
        0,
        0,
        0);

    int [] tagsToConsider = new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
    LimelightHelpers.SetFiducialIDFiltersOverride("", tagsToConsider);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateRawFiducials();
  }

  public void updateRawFiducials() {
    // LimelightHelpers.getRawFiducials() returns an empty array if no tags are detected,
    // so no NullPointerException will be thrown.
    rawFiducials = LimelightHelpers.getRawFiducials(limelightName);
  }

  public RawFiducial[] getRawFiducials() {
    return rawFiducials;
  }

  public boolean hasFiducials() {
    // Check if the array has any tags
    return rawFiducials != null && rawFiducials.length > 0;
  }

  /* Find the FIRST detected fiducial and return it ID */
  public int getFirstFiducialId() {
    // Return the ID of the first detected tag
    if (hasFiducials()) {
      return rawFiducials[0].id;
    }
    // Return a default value or throw an exception if no tags are present
    return -1; // -1 is a common indicator of no tag found
  }

  /* find the closest Fiducial and return the object */
  public RawFiducial getClosestFiducial() {
    if ( !hasFiducials() ) {
        return null;
    }

    RawFiducial closest = rawFiducials[0];
    double minDistance = closest.ta;

    for (RawFiducial fiducial : rawFiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }
    SmartDashboard.putNumber("minDistance", minDistance);
    return closest;
  }

  /* Check for a fiducial with a given ID */
  public RawFiducial getFiducialWithId(int id) {
  
    for (RawFiducial fiducial : rawFiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    return null;
  }

}
