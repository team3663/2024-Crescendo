package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
    private final LedIo io;

    private final LedInputsAutoLogged inputs = new LedInputsAutoLogged();

    private LedState desiredLedState = new LedState(0, 0, 0);

    public Led(LedIo io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Led", inputs);
        io.setColor(desiredLedState.red, desiredLedState.green, desiredLedState.blue);
    }
}
