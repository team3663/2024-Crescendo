package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionIo[] ios;
    private final VisionInputsAutoLogged[] cameraInputs;

    public Vision(VisionIo[] io) {
        this.ios = io;
        cameraInputs = new VisionInputsAutoLogged[ios.length];
        for(int i = 0; i < cameraInputs.length - 1; i++) {
            cameraInputs[i] = new VisionInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for(int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(cameraInputs[i]);
            Logger.processInputs("Vision", cameraInputs[i]);
        }
    }
}
