package frc.robot.config;

import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.led.LedIo;

public interface RobotFactory {
    default DrivetrainIO createDrivetrainIO() {
        return new DrivetrainIO() {
        };
    }

    default IntakeIO createIntakeIO() {
        return new IntakeIO() {
        };
    }

    default LedIo createLedIo() {
        return new LedIo() {
        };
    }
}
