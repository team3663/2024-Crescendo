package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LedColor;

public class RobotMode {
    public final LedColor red = new LedColor(255,0,0);
    public final LedColor yellow = new LedColor(255,255, 0);
    public final LedColor blue = new LedColor(0,0,255);
    private final LedColor black = new LedColor(0, 0, 0);


    public enum Mode {
        SPEAKER,
        AMP,
        TRAP
    }

    private static Mode mode;

    public RobotMode(Mode mode) {
        this.mode = mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public static Mode getMode() {
        return mode;
    }

    public LedColor getLedColor(Mode mode) {
        switch(mode) {
            case SPEAKER:
                return blue;
            case AMP:
                return yellow;
            case TRAP:
                return red;
            default:
                return black;
        }
    }

}
