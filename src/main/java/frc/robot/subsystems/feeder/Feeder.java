package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    private final FeederIO io;
    private final FeederIO.Inputs inputs = new FeederIO.Inputs();

    public Feeder(FeederIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    public Command runWithVoltage(double voltage) {
        return runEnd(
                () -> io.setVoltage(voltage),
                () -> io.setVoltage(0.0)
        );
    }
}