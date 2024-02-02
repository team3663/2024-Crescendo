package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Led extends SubsystemBase {
    private final LedIO io;

    private final LedIO.Inputs inputs = new LedIO.Inputs();

    private LedState desiredLEDState = new LedState(0, 0, 0);

    public Led(LedIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs, desiredLEDState.red, desiredLEDState.green, desiredLEDState.blue);
        io.setLEDs(desiredLEDState.red, desiredLEDState.green, desiredLEDState.blue);
    }

}
