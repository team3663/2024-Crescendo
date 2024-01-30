package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LEDIO;


public class LED extends SubsystemBase {
    private final LEDIO io;

    private final LEDIO.Inputs inputs = new LEDIO.Inputs();

    private LEDState desiredLEDState = new LEDState(0, 0, 0);

    public LED(LEDIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs, desiredLEDState.red, desiredLEDState.green, desiredLEDState.blue);
        io.setLEDs(desiredLEDState.red, desiredLEDState.green, desiredLEDState.blue);
    }

}
