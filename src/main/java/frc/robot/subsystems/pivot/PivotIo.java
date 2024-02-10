package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIo {

    default void updateInputs(PivotInputs inputs) {}

    /**
     * Sets a desired pivot angle
     * @param rad The target pivot angle
     */
    default void setTargetAngle(double rad) {}

    /**
     * Sets the sensor position
     * @param position The new position in radians
     */
    default void resetPosition(double position) {}

    /**
     * Runs the motor at given voltage
     * @param voltage The voltage motor runs on
     */
    default void setVoltage(double voltage) {}

    /**
     * Brakes the pivot at position
     */
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
