package frc.robot.config;

import frc.robot.subsystems.climber.ClimberIo;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.feeder.FeederIo;
import frc.robot.subsystems.intake.IntakeIo;
import frc.robot.subsystems.led.LedIo;
import frc.robot.subsystems.pivot.PivotIo;
import frc.robot.subsystems.shooter.ShooterIo;
import frc.robot.subsystems.vision.VisionIo;

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

    default ShooterIo createShooterIo() {
        return new ShooterIo() {
        };
    }

    default FeederIo createFeederIo() {
        return new FeederIo() {
        };
    }

    default ClimberIo createClimberIo() {
        return new ClimberIo() {
        };
    }

    default VisionIo[] createVisionIo() {
        return new VisionIo[]{
        };
    }
}
