package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedIo {
    default void updateInputs(LedInputs inputs) {
    }

    default void setLeds(int red, int green, int blue) {
    }

    @AutoLog
    class LedInputs {
        public long red;
        public long green;
        public long blue;
    }
}
