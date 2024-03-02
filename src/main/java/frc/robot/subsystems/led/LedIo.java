package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.StrobeAnimation;
import org.littletonrobotics.junction.AutoLog;

public interface LedIo {
    default void updateInputs(LedInputs inputs) {}

    default void setColor(LedColor color) {}

    default void setAnimation(Animation animation) {}

    @AutoLog
    class LedInputs {
        public double current;
        public double temperature;
    }
}
