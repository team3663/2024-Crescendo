package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class c2024VisionIo implements VisionIo {

    private final PhotonCamera camera;

    private double targetID;

    private double lastTargetYawDeg;

    public c2024VisionIo(PhotonCamera camera) {
        this.camera = camera;
    }

    public void updateInputs(Inputs inputs) {
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


    // Set the target ID
    public void setTargetID(int targetID) {
        this.targetID = targetID;
    }

    // Get list of targets
    public List<PhotonTrackedTarget> getTargetList() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getTargets();
        } else {
            return List.of();
        }
    }
}
