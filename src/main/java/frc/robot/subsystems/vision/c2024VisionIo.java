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

    public c2024VisionIo(PhotonCamera camera, Transform3d cameraOffsets) {
        this.camera = camera;
        this.cameraOffsets = cameraOffsets;

        estimator = new PhotonPoseEstimator(fieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraOffsets);
    }

    public void updateInputs(VisionInputs inputs) {
        double targetYawDeg = 0;
        boolean targetFound = false;
        List<PhotonTrackedTarget> targetList = getTargetList();
        for (PhotonTrackedTarget t : targetList){
            if (t.getFiducialId() == targetID){
                targetYawDeg = lastTargetYawDeg = t.getYaw();
                targetFound = true;
            }
        }
        inputs.tagFound = targetFound;
        if (targetFound) {
            inputs.tagYawRad = Units.degreesToRadians(targetYawDeg);
        } else {
            inputs.tagYawRad = Units.degreesToRadians(lastTargetYawDeg);
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

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        estimator.setReferencePose(prevEstimatedRobotPose);
        return estimator.update();
    }
}
