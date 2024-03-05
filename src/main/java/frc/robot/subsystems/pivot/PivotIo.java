package frc.robot.subsystems.pivot;

import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIo {

    default Pivot.Constants getConstants() {
        return new Pivot.Constants(0.0, Units.degreesToRadians(90.0), 0.0, -1.0);
    }

    default void updateInputs(PivotInputs inputs) {
    }

    /**
     * Sets a desired pivot angle
     *
     * @param rad The target pivot angle
     */
    default void setTargetAngle(double rad) {
    }

    /**
     * Sets the sensor position
     *
     * @param position The new position in radians
     */
    default void resetPosition(double position) {
    }

    /**
     * Runs the motor at given voltage
     *
     * @param voltage The voltage motor runs on
     */
    default void setVoltage(double voltage) {
    }

    /**
     * Brakes the pivot at position
     */
    default void stop() {
    }

    @AutoLog
    class PivotInputs {
        public double angle;
        public double angularVelocity;
        public double appliedVolts;
        public double currentDrawAmps;
        public double motorTemp;
    }
}
