package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LEDIO {
    default void updateInputs(Inputs inputs, int red, int green, int blue) {
        inputs.red = red;
        inputs.green = green;
        inputs.blue = blue;
    }

    default void setLEDs(int red, int green, int blue) {}

    @AutoLog
    class Inputs {
        public long red;
        public long green;
        public long blue;
    }
}
