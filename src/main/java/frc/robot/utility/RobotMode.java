package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotMode {
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

}
