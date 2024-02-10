package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface PivotIo {

    default void updateInputs(PivotInputs inputs) {}

    default void setTargetAngle(double rad) {}

    @AutoLog
    class PivotInputs {
        public double inputVoltageRight = 0;
        public double inputVoltageLeft = 0;
        public double outputVoltageRight = 0;
        public double outputVoltageLeft = 0;
        public double currentAngleRad = 0;
        public double currentVelocityRadPerSec = 0;
    }
}
