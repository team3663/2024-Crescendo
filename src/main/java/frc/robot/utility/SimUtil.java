package frc.robot.utility;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Utilities for simulations.
 */
public final class SimUtil {
    private SimUtil() {
        throw new UnsupportedOperationException();
    }

    /**
     * Limits the theoretical current draw of a simulated mechanism to the specified limit by decreasing the voltage.
     *
     * @param motor          The motor driving the mechanism.
     * @param motorVelocity  The velocity of the motor.
     * @param appliedVoltage The voltage applied to the motor.
     * @param maxCurrent     The maximum current the motor should draw.
     * @return The voltage to apply to the motor to stay within the current limit.
     */
    public static double applyCurrentLimit(DCMotor motor, double motorVelocity, double appliedVoltage, double maxCurrent) {
        double estCurrentDraw = motor.getCurrent(motorVelocity, appliedVoltage);
        if (Math.abs(estCurrentDraw) > maxCurrent) {
            double limitedTorque = motor.getTorque(Math.copySign(maxCurrent, estCurrentDraw));
            return motor.getVoltage(limitedTorque, motorVelocity);
        }

        return appliedVoltage;
    }
}
