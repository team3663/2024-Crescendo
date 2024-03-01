package frc.robot.subsystems.led;

import frc.robot.utility.RobotMode;
import org.littletonrobotics.junction.AutoLog;

import static java.awt.Color.blue;
import static java.awt.Color.yellow;
import static java.awt.Color.red;

public interface LedIo {
    default void updateInputs(LedInputs inputs) {}

    default void setColor(LedColor color) {}

    


    @AutoLog
    class LedInputs {
        public double current;
        public double temperature;
    }
}
