package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public interface VisionIo {
    default void updateInputs(VisionInputs inputs) {}

    @AutoLog
    class VisionInputs {
        public double tagYawRad = 0;
        public boolean tagFound = false;
        public int tagID = 0;
        public Pose3d estimatedPose = new Pose3d();
        public double timestampSeconds = 0;
        public List<PhotonTrackedTarget> trackedTargets;
        public boolean poseUpdated = false;
    }
}
