package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIo {

    default void updateInputs(PivotInputs inputs) {}

    default void setTargetAngle(double rad) {}

    default void resetPosition(double position) {}

    default void setVoltage(double voltage) {}

    default void stop(){}

    @AutoLog
    class PivotInputs {
        public double inputVoltagePrimary = 0;
        public double inputVoltageSecondary = 0;
        public double outputVoltagePrimary = 0;
        public double outputVoltageSecondary = 0;
        public double currentAngleRad = 0;
        public double currentVelocityRadPerSec = 0;
    }
}
