package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;

public class LedCandleIo implements LedIo {

    private final CANdle candle;

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
}
