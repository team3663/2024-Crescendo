package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroscopeIO {
    void updateInputs(Inputs inputs);

    class Inputs implements LoggableInputs {
        public Rotation2d yaw = new Rotation2d();

        @Override
        public void toLog(LogTable table) {
            table.put("Yaw", yaw);
        }

        @Override
        public void fromLog(LogTable table) {
            yaw = table.get("Yaw", new Rotation2d());
        }
    }
}
