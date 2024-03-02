package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {

    public enum Pattern {
        SOLID,
        FADE,
        LARSON,
        STROBE;
    }
    private final double ANIMATION_SPEED = 0.25;
    private final double LED_BRIGHTNESS = 0.01;
    private final int NUM_LEDS = 8;
    private final int POCKET_SIZE = 3;

    private final LedIo io;
    private final LedColor black = new LedColor(0, 0, 0);
    private final LedInputsAutoLogged inputs = new LedInputsAutoLogged();
    private Animation currentAnimation;
    private LedColor currentColor;
    private Pattern currentPattern = Pattern.SOLID;
    private final Animation startAnimation = new RgbFadeAnimation(LED_BRIGHTNESS, ANIMATION_SPEED, NUM_LEDS);

    public Led(LedIo io) {
        this.io = io;
        io.setAnimation(startAnimation);
        //io.setColor(black);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Led", inputs);
    }

    public void setColor(LedColor color) {
        currentColor = color;
        setPattern(currentPattern);
    }


    public void setPattern (Pattern pattern) {
        switch(pattern) {

            case SOLID:
                currentAnimation = null;
                io.setAnimation(currentAnimation);
                io.setColor(currentColor);
                currentPattern = Pattern.SOLID;
                break;

            case FADE:
                io.setColor(currentColor);
                currentAnimation = new RgbFadeAnimation(LED_BRIGHTNESS, ANIMATION_SPEED, NUM_LEDS);
                io.setAnimation(currentAnimation);
                currentPattern = Pattern.FADE;

            case LARSON:
                io.setColor(currentColor);
                currentAnimation = new LarsonAnimation(currentColor.red, currentColor.green, currentColor.blue, 0, ANIMATION_SPEED, NUM_LEDS, LarsonAnimation.BounceMode.Center , POCKET_SIZE);
                io.setAnimation(currentAnimation);
                currentPattern = Pattern.LARSON;
                break;

            case STROBE:
                io.setColor(currentColor);
                currentAnimation = new StrobeAnimation(currentColor.red, currentColor.green, currentColor.blue, 0, ANIMATION_SPEED, NUM_LEDS);
                io.setAnimation(currentAnimation);
                currentPattern = Pattern.STROBE;
                break;
        }
    }

    public Command setLedColor(LedColor color) {
        return runOnce(
                () -> io.setColor(color)
        );
    }
}
