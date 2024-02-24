package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface VisionIo {
    default void updateInputs(VisionInputs inputs) {}

    @AutoLog
    class VisionInputs {
        public double tagYawRad = 0;
        public boolean tagFound = false;
        public int tagID = 0;
        public EstimatedRobotPose estimatedPose;
        public boolean poseUpdated = false;
    }
}
