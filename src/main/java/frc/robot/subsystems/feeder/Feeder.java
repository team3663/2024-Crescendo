package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    private final FeederIo io;
    private final FeederInputsAutoLogged inputs = new FeederInputsAutoLogged();

    public Feeder(FeederIo io) {
        this.io = new LoggingFeederIo(io);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    public boolean isDetected() {
        return inputs.beamBreakSignaled;
    }
    public boolean isNotDetected() {
        return false;
//        return !inputs.beamBreakSignaled;
    }

    public Command runWithVoltage(double voltage) {
        return runEnd(
                () -> io.setVoltage(voltage),
                () -> io.setVoltage(0.0)
        );
    }
}