package frc.robot.utility;

import frc.robot.Robot;
import java.util.Set;

public class RobotIdUtil {

    public enum RobotId {
        SIM,
        C2023,
        C2024
    }

    private static final String C2023_MAC_ADDRESS = "00-80-2f-33-d0-3f";
    private static final String C2024_MAC_ADDRESS = "00-80-2f-33-d0-1b";

    static public RobotId getRobotId() {
        // If we are running in the simulator then return the SIM robot Id
        if (Robot.isSimulation()) {
            return RobotId.SIM;
        }

        Set<String> macAddresses = MacAddressUtil.getMacAddresses();
        if (!macAddresses.isEmpty()) {
            String macAddress = String.valueOf(macAddresses.stream().findFirst());

            return switch (macAddress) {
                case C2023_MAC_ADDRESS -> RobotId.C2023;
                case C2024_MAC_ADDRESS -> RobotId.C2024;
                default -> RobotId.C2024;
            };
        }

        return RobotId.C2024;
    }
}