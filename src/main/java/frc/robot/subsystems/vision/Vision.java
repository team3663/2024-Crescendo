package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class Vision extends SubsystemBase {
    private final VisionIo left_cam_io;
    private final VisionIo right_cam_io;
    private final VisionIo.Inputs left_cam_inputs = new VisionIo.Inputs();
    private final VisionIo.Inputs right_cam_inputs = new VisionIo.Inputs();

    public Vision(VisionIo left_cam_io, VisionIo right_cam_io) {
        this.left_cam_io = left_cam_io;
        this.right_cam_io = right_cam_io;
    }

    /**
     * @return A boolean statement on whether the target tag is found or not
     */
    public boolean getTargetFound() {
        return left_cam_inputs.tagFound || right_cam_inputs.tagFound;
    }

    /**
     * @return The yaw value between cameras and desired target in radians
     */
    public double getTargetYaw() {
        return (left_cam_inputs.tagYawRad + right_cam_inputs.tagYawRad) / 2;
    }

    /**
     * @return The ID of the desired target if both cameras have the same target; 0 if none are found
     */
    public int getTargetID() {
        if (left_cam_inputs.tagID == right_cam_inputs.tagID) {
            return left_cam_inputs.tagID;
        } else {
            return 0;
        }
    }

    @Override
    public void periodic() {
        left_cam_io.updateInputs(left_cam_inputs);
        right_cam_io.updateInputs(right_cam_inputs);
        Logger.processInputs("Vision", left_cam_inputs);
        Logger.processInputs("Vision", right_cam_inputs);
    }
}
