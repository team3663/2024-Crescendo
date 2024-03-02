package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIo {
    default void updateInputs(ShooterInputs inputs) {
    }

    default void setTargetVelocity(double velocity) {
    }

    default void setVoltage(double voltage) {
    }

    default void stop() {
    }

    @AutoLog
    class ShooterInputs {
        public double upperAngularVelocity;
        public double upperAppliedVolts;
        public double upperCurrentDrawAmps;
        public double upperMotorTemp;

        public double lowerAngularVelocity;
        public double lowerAppliedVolts;
        public double lowerCurrentDrawAmps;
        public double lowerMotorTemp;
    }
}
