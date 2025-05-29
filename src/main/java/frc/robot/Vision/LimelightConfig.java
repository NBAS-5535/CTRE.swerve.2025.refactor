package frc.robot.Vision;

//import edu.wpi.first.math.geometry.Pose3d;

public record LimelightConfig(String name, double forward, double side, double up, double roll, double pitch,
        double yaw) {
        /* 
        public LimelightConfig(String name, Pose3d pose3d) {
                this(name, pose3d.getX(), pose3d.getY(), pose3d.getZ(), pose3d.getRotation().getX(), pose3d.getRotation().getY(),
                        pose3d.getRotation().getZ());
                }
        */
}