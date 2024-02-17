package frc.robot.utility;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.Robot;

public class AdvantageKitHelper {

    /**
     * setupLogger - Configure AdvantageKit logging for the robot.
    */
    public static void setupLogger() {

        // Setup the network tables receiver.
        Logger.addDataReceiver(new NT4Publisher());

        // If this is a physical robot (with a Rio) then we can log to a USB drive.
        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        }
    }
}
