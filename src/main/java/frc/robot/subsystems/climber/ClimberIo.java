package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIo {
    default Climber.Constants getConstants() {
        return new Climber.Constants(0, -2.0);
    }

    default void updateInputs(ClimberInputs inputs) {
    }

    default void stop() {
        setVoltage(0.0, 0.0);
    }

    default void resetPosition() {
    }

    default void setTargetPosition(double leftHeight, double rightHeight) {
    }

    default void setVoltage(double voltageLeft, double voltageRight) {
    }

    default void setLocked(boolean lockedLeft, boolean lockedRight) {
    }

    @AutoLog
    class ClimberInputs {
        public double leftPosition;
        public double leftVelocity;
        public double leftAppliedVolts;
        public double leftCurrentDrawAmps;
        public double leftMotorTemp;
        public boolean leftLocked;

        public double rightPosition;
        public double rightVelocity;
        public double rightAppliedVolts;
        public double rightCurrentDrawAmps;
        public double rightMotorTemp;
        public boolean rightLocked;

    }
}
