package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveModuleIO {
    void updateInputs(Inputs inputs);

    void setTargetState(SwerveModuleState state);

    class Inputs implements LoggableInputs {
        public double steerAngleRad;
        public double encoderAngleRad;

        @Override
        public void toLog(LogTable table) {
            table.put("SteerMotorAngle", steerAngleRad);
            table.put("SteerEncoderAngle", encoderAngleRad);
        }

        @Override
        public void fromLog(LogTable table) {
            steerAngleRad = table.get("SteerMotorAngle", 0.0);
            encoderAngleRad = table.get("SteerEncoderAngle", 0.0);
        }
    }
}
