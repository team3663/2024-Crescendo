package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionIo io;
    private final VisionIo.Inputs inputs = new VisionIo.Inputs();

    public Vision(VisionIo io) {
        this.io = io;
    }

    /**
     * @return A boolean statement on whether the target tag is found or not
     */
    public boolean getTargetFound() {
        return inputs.tagFound;
    }

    /**
     * @return The yaw value between cameras and desired target in radians
     */
    public double getTargetYaw() {
        return inputs.tagYawRad;
    }

    /**
     * @return The ID of the desired target
     */
    public int getTargetID() {
        return inputs.tagID;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }
}
