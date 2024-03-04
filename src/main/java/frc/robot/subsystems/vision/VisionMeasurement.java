package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionMeasurement {
    public Pose3d estimatedPose;
    public double timestamp;

    public VisionMeasurement(Pose3d pose, double timestamp) {
        this.estimatedPose = pose;
        this.timestamp = timestamp;
    }
}
