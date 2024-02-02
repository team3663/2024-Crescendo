package frc.robot.subsystems.led;

public class LedState {
    public static final LedState OFF = new LedState(0, 0, 0); // The LED lights are off
    public static final LedState GET_NOTE = new LedState(255, 165, 0); // The LED lights are orange
    public static final LedState SHOOT_NOTE = new LedState(0, 255, 0); // The LED lights are green
    public static final LedState LOW_BATTERY = new LedState(255, 0, 0); // The LED lights are red
    public static final LedState CLIMB = new LedState(155, 0, 255); // The LED lights are purple

    private LedState() {}

    public LedState(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public void copyFrom(LedState other) {
        this.red = other.red;
        this.green = other.green;
        this.blue = other.blue;
    }

    public int red;
    public int green;
    public int blue;
}
