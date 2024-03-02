package frc.robot.utility;

import frc.robot.Robot;

import java.util.Optional;
import java.util.Set;

public class RobotIdUtil {

    public enum RobotId {
        SIM,
        BENCH_RIO1,
        BENCH_RIO2,
        C2023,
        C2024
    }

    private static final String BENCH_RIO1_MAC_ADDRESS = "00-80-2f-21-c5-21";
    private static final String BENCH_RIO2_MAC_ADDRESS = "00-80-2f-24-4b-42";
    private static final String C2023_MAC_ADDRESS = "00-80-2f-33-d0-3f";
    private static final String C2024_MAC_ADDRESS = "00-80-2f-33-d0-1b";

    static public RobotId getRobotId() {
        // If we are running in the simulator then return the SIM robot Id
        if (Robot.isSimulation()) {
            return RobotId.SIM;
        }

        Set<String> macAddresses = MacAddressUtil.getMacAddresses();
        if (!macAddresses.isEmpty()) {
            String macAddress = macAddresses.stream().findFirst().map(Object::toString).orElse("");
            System.out.println("-------- Found MAC address: " + macAddress);

            return switch (macAddress) {
                case BENCH_RIO1_MAC_ADDRESS -> RobotId.BENCH_RIO1;
                case BENCH_RIO2_MAC_ADDRESS -> RobotId.BENCH_RIO2;
                case C2023_MAC_ADDRESS -> RobotId.C2023;
                case C2024_MAC_ADDRESS -> RobotId.C2024;
                default -> RobotId.C2024;
            };
        }

        return RobotId.C2024;
    }
}