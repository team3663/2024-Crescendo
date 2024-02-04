package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
    private final LedIo io;
    private final LedColor black = new LedColor(0, 0, 0);

    private final LedInputsAutoLogged inputs = new LedInputsAutoLogged();

    public Led(LedIo io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Led", inputs);
    }

    public void setColor(LedColor color) {
        io.setColor(color);
    }

    public Command setLedColor(LedColor color) {
        return runEnd(
                () -> io.setColor(color),
                () -> io.setColor(black)
        );
    }
}
