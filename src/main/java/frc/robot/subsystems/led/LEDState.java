package frc.robot.subsystems.led;

public class LEDState {
    public static final LEDState OFF = new LEDState(0, 0, 0); // The LED lights are off
    public static final LEDState GET_NOTE = new LEDState(255, 165, 0); // The LED lights are orange
    public static final LEDState SHOOT_NOTE = new LEDState(0, 255, 0); // The LED lights are green
    public static final LEDState LOW_BATTERY = new LEDState(255, 0, 0); // The LED lights are red
    public static final LEDState CLIMB = new LEDState(155, 0, 255); // The LED lights are purple

6

    private LEDState() {}

    public LEDState(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public void copyFrom(LEDState other) {
        this.red = other.red;
        this.green = other.green;
        this.blue = other.blue;
    }

    public int red;
    public int green;
    public int blue;
}
