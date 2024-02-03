package frc.robot.config;

import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.SimpleSimDrivetrain;

public class SimRobotFactory implements RobotFactory {
    @Override
    public DrivetrainIO createDrivetrainIO() {
        return new SimpleSimDrivetrain();
    }
}
