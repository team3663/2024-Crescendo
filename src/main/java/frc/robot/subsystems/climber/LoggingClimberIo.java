package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

/**
 * Wrapper for {@link ClimberIo} that logs outputs.
 */
public final class LoggingClimberIo implements ClimberIo {
    private final ClimberIo inner;

    public LoggingClimberIo(ClimberIo inner) {
        this.inner = inner;

        Logger.recordOutput("Climber/Left/TargetPosition", Double.NaN);
        Logger.recordOutput("Climber/Right/TargetPosition", Double.NaN);
        Logger.recordOutput("Climber/Left/TargetVoltage", Double.NaN);
        Logger.recordOutput("Climber/Right/TargetVoltage", Double.NaN);
        Logger.recordOutput("Climber/Left/Zeroed", false);
        Logger.recordOutput("Climber/Right/Zeroed", false);
        Logger.recordOutput("Climber/Left/Locked", false);
        Logger.recordOutput("Climber/Right/Locked", false);
    }

    @Override
    public Climber.Constants getConstants() {
        return inner.getConstants();
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inner.updateInputs(inputs);
    }

    @Override
    public void stop() {
        Logger.recordOutput("Climber/Left/TargetVoltage", 0.0);
        Logger.recordOutput("Climber/Right/TargetVoltage", 0.0);

        inner.stop();
    }

    @Override
    public void resetPosition() {
        Logger.recordOutput("Climber/Left/Zeroed", true);
        Logger.recordOutput("Climber/Right/Zeroed", true);

        inner.resetPosition();
    }

    @Override
    public void setTargetPosition(double leftHeight, double rightHeight) {
        Logger.recordOutput("Climber/Left/TargetPosition", leftHeight);
        Logger.recordOutput("Climber/Right/TargetPosition", rightHeight);
        Logger.recordOutput("Climber/Left/TargetVoltage", Double.NaN);
        Logger.recordOutput("Climber/Right/TargetVoltage", Double.NaN);

        inner.setTargetPosition(leftHeight, rightHeight);
    }

    @Override
    public void setVoltage(double voltageLeft, double voltageRight) {
        Logger.recordOutput("Climber/Left/TargetPosition", Double.NaN);
        Logger.recordOutput("Climber/Right/TargetPosition", Double.NaN);
        Logger.recordOutput("Climber/Left/TargetVoltage", voltageLeft);
        Logger.recordOutput("Climber/Right/TargetVoltage", voltageRight);

        inner.setVoltage(voltageLeft, voltageRight);
    }

    @Override
    public void setLocked(boolean lockedLeft, boolean lockedRight) {
        Logger.recordOutput("Climber/Left/Locked", lockedLeft);
        Logger.recordOutput("Climber/Right/Locked", lockedRight);

        inner.setLocked(lockedLeft, lockedRight);
    }
}
