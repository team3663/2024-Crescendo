package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

public class LoggingPivotIo implements PivotIo {
    private final PivotIo inner;

    public LoggingPivotIo(PivotIo inner) {
        this.inner = inner;

        Logger.recordOutput("Pivot/TargetAngle", Double.NaN);
        Logger.recordOutput("Pivot/TargetVoltage", Double.NaN);
    }

    @Override
    public Pivot.Constants getConstants() {
        return inner.getConstants();
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        inner.updateInputs(inputs);
    }

    @Override
    public void resetPosition(double position) {
        inner.resetPosition(position);
    }

    @Override
    public void setTargetAngle(double rad) {
        Logger.recordOutput("Pivot/TargetAngle", rad);
        Logger.recordOutput("Pivot/TargetVoltage", Double.NaN);

        inner.setTargetAngle(rad);
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Pivot/TargetAngle", Double.NaN);
        Logger.recordOutput("Pivot/TargetVoltage", voltage);

        inner.setVoltage(voltage);
    }

    @Override
    public void stop() {
        Logger.recordOutput("Pivot/TargetAngle", Double.NaN);
        Logger.recordOutput("Pivot/TargetVoltage", Double.NaN);

        inner.stop();
    }
}
