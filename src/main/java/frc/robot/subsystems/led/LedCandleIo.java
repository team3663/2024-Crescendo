package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.Animation;


public class LedCandleIo implements LedIo {

    private final CANdle candle;

    private Animation animation;

    public LedCandleIo(CANdle candle) {
        this.candle = candle;
    }

    @Override
    public void updateInputs(LedInputs inputs) {
        inputs.current = candle.getCurrent();
        inputs.temperature = candle.getTemperature();
    }

    @Override
    public void setColor(LedColor color) {
        candle.setLEDs(color.red, color.green, color.blue);
    }
    @Override
    public void setAnimation(Animation animation) {
        candle.animate(animation);
    }

}
