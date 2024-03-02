package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
    private final double STROBE_SPEED = 0.5;
    private final int NUM_LEDS = 3;
    private final LedIo io;
    private final LedColor black = new LedColor(0, 0, 0);

    private final LedInputsAutoLogged inputs = new LedInputsAutoLogged();
    private LedColor currentColor;
    private Animation currentAnimation;

    public Led(LedIo io) {
        this.io = io;
    }

    public enum Pattern {
        SOLID,
        STROBE;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Led", inputs);
    }

    public void setColor(LedColor color) {
        currentColor = color;
        io.setColor(color);
    }

    public void setPattern (Pattern pattern) {
        switch(pattern) {
            case SOLID:
                currentAnimation = null;
                io.setAnimation(currentAnimation);
                break;

            case STROBE:
                currentAnimation = new StrobeAnimation(currentColor.red, currentColor.green, currentColor.blue, 0, STROBE_SPEED, NUM_LEDS);
                io.setAnimation(currentAnimation);
                break;
        }

    }

    public Command setLedColor(LedColor color) {
        return runEnd(
                () -> io.setColor(color),
                () -> io.setColor(black)
        );
    }

}
