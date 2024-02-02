package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LedIo;


public class Led extends SubsystemBase {
    private final LedIo io;

    private final LedIo.Inputs inputs = new LedIo.Inputs();

    private LedState desiredLedState = new LedState(0, 0, 0);

    public Led(LedIo io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs, desiredLedState.red, desiredLedState.green, desiredLedState.blue);
        io.setLEDs(desiredLedState.red, desiredLedState.green, desiredLedState.blue);
    }
}
