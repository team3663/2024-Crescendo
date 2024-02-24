package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class Vision extends SubsystemBase {
    private final VisionIo[] ios;
    private final VisionInputsAutoLogged[] visionInputs;

    public Vision(VisionIo[] io) {
        this.ios = io;
        visionInputs = new VisionInputsAutoLogged[ios.length];
        for(int i = 0; i < visionInputs.length - 1; i++) {
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

    public EstimatedRobotPose[] getEstimatedRobotPoses() {
        EstimatedRobotPose[] estimatedRobotPoses = new EstimatedRobotPose[2];
        for(int i = 0; i < 2; i++) {
            estimatedRobotPoses[i] = visionInputs[i].estimatedPose;
        }
        return estimatedRobotPoses;
    }

    public Command visionToDrivetrain() {
        return run();
    }
}
