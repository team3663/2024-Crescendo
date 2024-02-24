package frc.robot;

import frc.robot.subsystems.led.LedColor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;
    public static final LedColor OFF = new LedColor(0, 0, 0); // The LED lights are off
    public static final LedColor GET_NOTE = new LedColor(255, 125, 0); // The LED lights are yellow
    public static final LedColor HAS_NOTE = new LedColor(100, 255, 100); // The LED lights are green
    public static final LedColor EJECT_NOTE = new LedColor(255, 0, 0); // The LED lights are red
}
