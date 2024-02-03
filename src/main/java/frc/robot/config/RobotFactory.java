package frc.robot.config;

import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.intake.IntakeIo;

public interface RobotFactory {
    default DrivetrainIO createDrivetrainIO() {
        return new DrivetrainIO() {
        };
    }

    default IntakeIo createIntakeIo() {
        return new IntakeIo() {
        };
    }
}
