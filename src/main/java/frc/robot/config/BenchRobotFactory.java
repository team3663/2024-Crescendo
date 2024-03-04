package frc.robot.config;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.math.geometry.Transform3d;
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
        PhotonCamera camera = new PhotonCamera("");
        Transform3d cameraOffsets = new Transform3d();

        return new VisionIo[]{
                new c2024VisionIo(camera, cameraOffsets)
        };
    }
}
