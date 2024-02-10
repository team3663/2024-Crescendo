package frc.robot.config;

import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.intake.IntakeIo;
import frc.robot.subsystems.led.LedIo;
import frc.robot.subsystems.pivot.PivotIo;

public interface RobotFactory {
    default DrivetrainIO createDrivetrainIO() {
        return new DrivetrainIO() {
        };
    }

    default IntakeIo createIntakeIo() {
        return new IntakeIo() {
        };
    }

    default LedIo createLedIo() {
        return new LedIo() {
        };
    }

    default PivotIo createPivotIo() {
        return new PivotIo() {
        };
    }
}
