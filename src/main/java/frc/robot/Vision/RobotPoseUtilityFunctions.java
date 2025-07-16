// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class RobotPoseUtilityFunctions {

    public RobotPoseUtilityFunctions() {

    }

    public static Pose2d translateCoord(Pose2d originalPose, double degreesRotate, double distance) {
        double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
        double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);
    
        return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
    }
    
    public static double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
        return Math.sqrt(Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
    }

    public static Pose2d rotatePose(Pose2d originalPose, double deg) {
    return new Pose2d(originalPose.getX(), originalPose.getY(),
        new Rotation2d(Units.degreesToRadians(originalPose.getRotation().getDegrees() - deg)));
  }
}
