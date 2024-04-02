package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIo {
    default Constants getConstants() {
        return new Constants("Unnamed");
    }

    default void updateInputs(VisionInputs inputs) {
    }

    record Constants(String name) {
    }

    @AutoLog
    class VisionInputs {
        public Pose3d estimatedPose = new Pose3d();
        public Pose2d estimatedPose2d = new Pose2d();
        public double timestampSeconds;
        public int[] targetIds;
        public boolean poseUpdated;
    }
}
