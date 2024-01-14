package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2Gyroscope implements GyroscopeIO {
    private final Pigeon2 gyroscope;

    public Pigeon2Gyroscope(Pigeon2 gyroscope) {
        this.gyroscope = gyroscope;

        Pigeon2Configuration config = new Pigeon2Configuration();
        gyroscope.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        StatusSignal<Double> yawSignal = gyroscope.getYaw();
        yawSignal.refresh();

        inputs.yaw = Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
    }
}
