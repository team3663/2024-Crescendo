package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private VisionIO.Inputs inputs = new VisionIO.Inputs();

    public Vision(VisionIO io) {
        this.io = io;
    }

    // Returns
    public boolean getTargetFound() {
        return inputs.tagFound;
    }

    // Returns target tag yaw in radians
    public double getTargetYaw() {
        return inputs.tagYawRad;
    }

    // Get the target ID
    public int getTargetID() {
        return inputs.tagID;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }
}
