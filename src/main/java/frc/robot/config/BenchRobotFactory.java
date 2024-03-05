package frc.robot.config;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionIo;
import frc.robot.subsystems.vision.C2024VisionIo;
import org.photonvision.PhotonCamera;

public class BenchRobotFactory implements RobotFactory {
/*    @Override
    public LedIo createLedIo() {
        CANdle candle = new CANdle(1);
        return new LedCandleIo(candle);
    }*/

    @Override
    public VisionIo[] createVisionIo() {
        String cameraName = "Arducam_OV9782";
        Transform3d cameraTransform = new Transform3d(
                new Translation3d(2.0, 0.0, 1.0),
                new Rotation3d(0.0, 0.0, 0.0)
        );

        return new VisionIo[]{
                new C2024VisionIo(cameraName, cameraTransform)
        };
    }
}
