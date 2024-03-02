package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

public class LoggingShooterIo implements ShooterIo {
    private final ShooterIo inner;

    public LoggingShooterIo(ShooterIo inner) {
        this.inner = inner;

        Logger.recordOutput("Shooter/TargetVelocity", Double.NaN);
        Logger.recordOutput("Shooter/TargetVoltage", Double.NaN);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inner.updateInputs(inputs);
    }

    @Override
    public void setTargetVelocity(double velocity) {
        Logger.recordOutput("Shooter/TargetVelocity", velocity);
        Logger.recordOutput("Shooter/TargetVoltage", Double.NaN);

        inner.setTargetVelocity(velocity);
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Shooter/TargetVelocity", Double.NaN);
        Logger.recordOutput("Shooter/TargetVoltage", voltage);

        inner.setVoltage(voltage);
    }

    @Override
    public void stop() {
        Logger.recordOutput("Shooter/TargetVelocity", Double.NaN);
        Logger.recordOutput("Shooter/TargetVoltage", Double.NaN);

        inner.stop();
    }
}
