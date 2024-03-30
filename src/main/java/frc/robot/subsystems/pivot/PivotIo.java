package frc.robot.subsystems.pivot;

import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIo {

    default Pivot.Constants getConstants() {
        return new Pivot.Constants(
                0.0, Units.degreesToRadians(90.0), 0.0, -1.0,
                0.0, Units.degreesToRadians(90.0), 0.0, -1.0,
                Units.degreesToRadians(80.0), Units.degreesToRadians(70.0));
    }

    default void updateInputs(PivotInputs inputs) {
    }

    /**
     * Sets a desired pivot angle
     *
     * @param targetPivotAngle The target pivot angle
     * @param targetAmpAngle   The target amp angle
     */
    default void setTargetAngle(double targetPivotAngle, double targetAmpAngle) {
    }

    /**
     * Sets the sensor position
     *
     * @param pivotPosition The new position in radians
     */
    default void resetPivotPosition(double pivotPosition) {
    }

    /**
     * Sets the sensor position
     *
     * @param ampPosition The new position in radians
     */
    default void resetAmpPosition(double ampPosition) {
    }

    /**
     * Runs the motor at given voltage
     *
     * @param pivotVoltage The voltage motor runs on
     */
    default void setVoltage(double pivotVoltage, double ampVoltage) {
    }

    /**
     * Brakes the pivot at position
     */
    default void stop() {
        setVoltage(0.0, 0.0);
    }

    @AutoLog
    class PivotInputs {
        public double pivotAngle;
        public double pivotAngularVelocity;
        public double pivotAppliedVolts;
        public double pivotCurrentDrawAmps;
        public double pivotMotorTemp;

        public double ampAngle;
        public double ampAngularVelocity;
        public double ampAppliedVolts;
        public double ampCurrentDrawAmps;
        public double ampMotorTemp;
    }
}
