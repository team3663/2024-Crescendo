package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedIo {
    default void updateInputs(LedInputs inputs) {}

    default void setColor(LedColor color) {}

    @AutoLog
    class LedInputs {
        public double current;
        public double temperature;
    }
}
