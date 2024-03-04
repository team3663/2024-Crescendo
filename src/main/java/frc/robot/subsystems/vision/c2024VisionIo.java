package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class c2024VisionIo implements VisionIo {

    private final PhotonPoseEstimator estimator;

    public c2024VisionIo(PhotonCamera camera, Transform3d cameraOffsets) {
        AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        estimator = new PhotonPoseEstimator(fieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraOffsets);
    }

    public void updateInputs(VisionInputs visionInputs) {

        // Assume pose will not be updated.
        visionInputs.poseUpdated = false;

        // Values used by the CLOSEST_TO_LAST_POSE and CLOSEST_TO_REFERENCE_POST strategies.
        estimator.setLastPose(visionInputs.estimatedPose);
        estimator.setReferencePose(visionInputs.estimatedPose);

        Optional<EstimatedRobotPose> newPose = estimator.update();

        // If there is no new pose then we have nothing to do, just bail out.
        if (newPose.isEmpty()) {
            return;
        }

        EstimatedRobotPose newEstimate = newPose.get();
        visionInputs.estimatedPose = newEstimate.estimatedPose;
        visionInputs.timestampSeconds = newEstimate.timestampSeconds;

        // Get the list of targets used in the current estimate and store their Ids in our Inputs.
        List<PhotonTrackedTarget> targets = newEstimate.targetsUsed;
        int[] ids = new int[targets.size()];
        int index = 0;
        for (PhotonTrackedTarget target : targets) {
            ids[index++] = target.getFiducialId();
        }
        visionInputs.targetIds = ids;
        visionInputs.poseUpdated = true;
    }
}
