package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

public class LoggingFeederIo implements FeederIo {
    private final FeederIo inner;

    public LoggingFeederIo(FeederIo inner) {
        this.inner = inner;

        Logger.recordOutput("Feeder/TargetVoltage", Double.NaN);
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        inner.updateInputs(inputs);
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Feeder/TargetVoltage", voltage);

        inner.setVoltage(voltage);
    }
}
