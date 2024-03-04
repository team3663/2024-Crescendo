package frc.robot.config;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.climber.ClimberIo;
import frc.robot.subsystems.climber.SimClimberIo;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.SimpleSimDrivetrain;
import frc.robot.subsystems.led.LedCandleIo;
import frc.robot.subsystems.led.LedIo;
import frc.robot.subsystems.shooter.ShooterIo;
import frc.robot.subsystems.shooter.SimShooterIo;
import frc.robot.subsystems.vision.VisionIo;
import frc.robot.subsystems.vision.c2024VisionIo;
import org.photonvision.PhotonCamera;

public class BenchRobotFactory implements RobotFactory {
/*    @Override
    public LedIo createLedIo() {
        CANdle candle = new CANdle(1);
        return new LedCandleIo(candle);
    }*/

    @Override
    public VisionIo[] createVisionIo() {
        PhotonCamera camera = new PhotonCamera("Arducam_OV9782");
        Transform3d cameraOffsets = new Transform3d(
                new Translation3d(2.0, 0.0, 1.0),
                new Rotation3d(0.0, 0.0, 0.0)
        );

        return new VisionIo[]{
                new c2024VisionIo(camera, cameraOffsets)
        };
    }
}
