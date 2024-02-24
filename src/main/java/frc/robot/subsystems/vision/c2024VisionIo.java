package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class c2024VisionIo implements VisionIo {

    private final PhotonCamera camera;

    private double targetID;

    private double lastTargetYawDeg;

    private final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final Transform3d cameraOffsets;

    private final PhotonPoseEstimator estimator;

    private Pose2d previousPose = new Pose2d();

    public c2024VisionIo(PhotonCamera camera, Transform3d cameraOffsets) {
        this.camera = camera;
        this.cameraOffsets = cameraOffsets;

        estimator = new PhotonPoseEstimator(fieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraOffsets);
    }

    public void updateInputs(VisionInputs visionInputs) {
        double targetYawDeg = 0;
        boolean targetFound = false;
        List<PhotonTrackedTarget> targetList = getTargetList();

        // Iterates through every tracked tag and see if their ID is the same as the ID of the target tag
        for (PhotonTrackedTarget t : targetList){
            if (t.getFiducialId() == targetID){
                targetYawDeg = lastTargetYawDeg = t.getYaw();
                targetFound = true;
            }
        }
        visionInputs.tagFound = targetFound;

        if (targetFound) {
            // Target yaw is updated if target is found
            visionInputs.tagYawRad = Units.degreesToRadians(targetYawDeg);

        } else {
            // Target yaw is not updated if target is not found
            visionInputs.tagYawRad = Units.degreesToRadians(lastTargetYawDeg);
        }

        if(getEstimatedGlobalPose().isPresent() && getEstimatedGlobalPose().get().estimatedPose.toPose2d() != previousPose) {
            // Pose is updated and the previous pose is now set to the new recorded pose
            visionInputs.estimatedPose = getEstimatedGlobalPose().get().estimatedPose;
            visionInputs.timestampSeconds = getEstimatedGlobalPose().get().timestampSeconds;
            visionInputs.trackedTargets = getEstimatedGlobalPose().get().targetsUsed;
            previousPose = visionInputs.estimatedPose.toPose2d();
            visionInputs.poseUpdated = true;

        } else {
            // Pose is not updated if pose is not seen or the same as the previous pose
            visionInputs.poseUpdated = false;
        }
    }


    /**
     * Sets the desired target to the one containing given ID
     * @param targetID The ID of the desired apriltag target
     */
    public void setTargetID(int targetID) {
        this.targetID = targetID;
    }

    /**
     * @return List of apriltag targets
     */
    public List<PhotonTrackedTarget> getTargetList() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getTargets();
        } else {
            return List.of();
        }
    }

    /**
     * @return Optional estimated robot pose
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return estimator.update();
    }
}
