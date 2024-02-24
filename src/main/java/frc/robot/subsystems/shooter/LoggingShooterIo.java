package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

public class LoggingShooterIo implements ShooterIo {
    private final ShooterIo inner;

    public LoggingShooterIo(ShooterIo inner) {
        this.inner = inner;

        Logger.recordOutput("Shooter/TargetVelocity", Double.NaN);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inner.updateInputs(inputs);
    }

    @Override
    public void setTargetVelocity(double velocity) {
        Logger.recordOutput("Shooter/TargetVelocity", velocity);

        inner.setTargetVelocity(velocity);
    }

    @Override
    public void stop() {
        Logger.recordOutput("Shooter/TargetVelocity", Double.NaN);

        inner.stop();
    }
}
