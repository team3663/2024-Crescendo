package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

public class LoggingPivotIo implements PivotIo {
    private final PivotIo inner;

    public LoggingPivotIo(PivotIo inner) {
        this.inner = inner;

        Logger.recordOutput("Pivot/TargetPivotAngle", Double.NaN);
        Logger.recordOutput("Pivot/TargetPivotVoltage", Double.NaN);
        Logger.recordOutput("Pivot/TargetAmpAngle", Double.NaN);
        Logger.recordOutput("Pivot/TargetAmpVoltage", Double.NaN);
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
    public void resetPivotPosition(double pivotPosition) {
        inner.resetPivotPosition(pivotPosition);
    }

    @Override
    public void resetAmpPosition(double ampPosition) {
        inner.resetAmpPosition(ampPosition);
    }

    @Override
    public void setTargetAngle(double targetPivotAngle, double targetAmpAngle) {
        Logger.recordOutput("Pivot/TargetPivotAngle", targetPivotAngle);
        Logger.recordOutput("Pivot/TargetPivotVoltage", Double.NaN);
        Logger.recordOutput("Pivot/TargetAmpAngle", targetAmpAngle);
        Logger.recordOutput("Pivot/TargetAmpVoltage", Double.NaN);

        inner.setTargetAngle(targetPivotAngle, targetAmpAngle);
    }

    @Override
    public void setVoltage(double pivotVoltage, double ampVoltage) {
        Logger.recordOutput("Pivot/TargetPivotAngle", Double.NaN);
        Logger.recordOutput("Pivot/TargetPivotVoltage", pivotVoltage);
        Logger.recordOutput("Pivot/TargetAmpAngle", Double.NaN);
        Logger.recordOutput("Pivot/TargetAmpVoltage", ampVoltage);

        inner.setVoltage(pivotVoltage, ampVoltage);
    }

    @Override
    public void stop() {
        Logger.recordOutput("Pivot/TargetPivotAngle", Double.NaN);
        Logger.recordOutput("Pivot/TargetPivotVoltage", Double.NaN);
        Logger.recordOutput("Pivot/TargetAmpAngle", Double.NaN);
        Logger.recordOutput("Pivot/TargetAmpVoltage", Double.NaN);

        inner.stop();
    }
}
