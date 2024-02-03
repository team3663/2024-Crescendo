package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedIo {
    default void updateInputs(LedInputs inputs) {}

    default void setColor(int red, int green, int blue) {}

    @AutoLog
    class LedInputs {
        public int red;
        public int green;
        public int blue;
    }
}
