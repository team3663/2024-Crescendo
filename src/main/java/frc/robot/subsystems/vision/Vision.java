package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {
    private final VisionIo[] ios;
    private final VisionInputsAutoLogged[] visionInputs;

    public Vision(VisionIo[] io) {
        this.ios = io;
        visionInputs = new VisionInputsAutoLogged[ios.length];
        for(int i = 0; i < visionInputs.length; i++) {
            visionInputs[i] = new VisionInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for(int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(visionInputs[i]);
            Logger.processInputs("Vision", visionInputs[i]);
        }
    }

    /**
     * @return List of estimated robot poses from each camera if the pose has been updated
     */
    public List<EstimatedRobotPose> getEstimatedRobotPoses() {
        List<EstimatedRobotPose> estimatedRobotPoses = new ArrayList<EstimatedRobotPose>();
        for(VisionInputsAutoLogged visionInput : visionInputs) {
            if(visionInput.poseUpdated) {
                estimatedRobotPoses.add(visionInput.estimatedPose);
            }
        }
        return estimatedRobotPoses;
    }

    /**
     * @param drivetrain Subsystem of the drivetrain the vision pose estimates would be sent
     * @return Command that adds vision measurements to the drivetrain
     */
    public Command visionToDrivetrain(Drivetrain drivetrain) {
        return run(
                () -> drivetrain.addVisionMeasurements(getEstimatedRobotPoses())
        );
    }
}
