package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Pivot extends SubsystemBase {
    private static final double ANGLE_THRESHOLD = Units.degreesToRadians(2.5);
    public static final double VELOCITY_WAIT_SEC = 0.1;
    public static final double VELOCITY_ZERO_RANGE = 0.01;

    private final PivotIo io;
    private final PivotInputsAutoLogged inputs = new PivotInputsAutoLogged();
    private final Constants constants;

    private boolean zeroed = false;

    public Pivot(PivotIo io) {
        this.io = new LoggingPivotIo(io);
        this.constants = io.getConstants();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
    }

    public Constants getConstants() {
        return constants;
    }

    public double getAngle() {
        return inputs.pivotAngle;
    }

    public boolean isZeroed() {
        return zeroed;
    }

    public Command follow(DoubleSupplier pivotAngleSupplier, DoubleSupplier ampAngleSupplier) {
        return run(() -> {
            double targetPivotAngle = pivotAngleSupplier.getAsDouble();
            double targetAmpAngle = ampAngleSupplier.getAsDouble();

            // Do nothing if not zeroed
            if (!zeroed) return;

            // Ignore if target angle is not finite
            if (!Double.isFinite(targetPivotAngle) || !Double.isFinite(targetAmpAngle)) return;

            if (targetPivotAngle > constants.pivotDangerAngle() ||
                    inputs.pivotAngle > constants.pivotDangerAngle()) {
                targetAmpAngle = Math.max(targetAmpAngle, constants.ampSafeAngle());

                if (inputs.ampAngle < constants.ampSafeAngle() - ANGLE_THRESHOLD)
                    targetPivotAngle = Math.min(targetPivotAngle, constants.pivotDangerAngle());
            }

            io.setTargetAngle(
                    MathUtil.clamp(targetPivotAngle, constants.minPivotAngle(), constants.maxPivotAngle()),
                    MathUtil.clamp(targetAmpAngle, constants.minAmpAngle(), constants.maxAmpAngle())
            );
        });
    }

    public Command follow(DoubleSupplier pivotAngleSupplier) {
        return follow(pivotAngleSupplier, constants::restingPivotAngle);
    }

    public Command moveTo(double pivotAngle) {
        return follow(() -> pivotAngle)
                .until(() -> Math.abs(pivotAngle - inputs.pivotAngle) < ANGLE_THRESHOLD);
    }

    public Command moveTo(double pivotAngle, double ampAngle) {
        return follow(() -> pivotAngle, () -> ampAngle)
                .until(() -> Math.abs(pivotAngle - inputs.pivotAngle) < ANGLE_THRESHOLD &&
                        Math.abs(ampAngle - inputs.ampAngle) < ANGLE_THRESHOLD);
    }

    public Command zero() {
        // voltages[0] is pivot voltage
        // voltages[1] is amp voltage
        double[] voltages = { 0.0, 0.0};

        Command zeroPivotCommand =
                // Wait for pivot motor to start moving before determining whether motor velocity is zero
                Commands.waitSeconds(VELOCITY_WAIT_SEC)
                        // Checks whether the motor velocity is zero after wait period is over
                        .andThen(Commands.waitUntil(() -> Math.abs(inputs.pivotAngularVelocity) < VELOCITY_ZERO_RANGE))
                        // Moves at a set voltage and stops moving when motor velocity is zero/reaches the hard stop
                        .deadlineWith(Commands.runEnd(() -> voltages[0] = constants.zeroPivotVoltage(), () -> voltages[0] = 0.0))
                        // Sensor position is reset to zero after pivot moves to its hard stop
                        .andThen(() -> io.resetPivotPosition(0));

        Command zeroAmpCommand =
                // Wait for pivot motor to start moving before determining whether motor velocity is zero
                Commands.waitSeconds(VELOCITY_WAIT_SEC*2.0)
                        // Checks whether the motor velocity is zero after wait period is over
                        .andThen(Commands.waitUntil(() -> Math.abs(inputs.ampAngularVelocity) < VELOCITY_ZERO_RANGE))
                        // Moves at a set voltage and stops moving when motor velocity is zero/reaches the hard stop
                        .deadlineWith(Commands.runEnd(() -> voltages[1] = constants.zeroAmpVoltage(), () -> voltages[1] = 0.0))
                        // Sensor position is reset to zero after pivot moves to its hard stop
                        .andThen(() -> io.resetAmpPosition(0));

        return Commands.parallel(
                        zeroPivotCommand,
                        zeroAmpCommand
                )
                // This commands actually uses the voltages array to set the motor voltages
                .deadlineWith(runEnd(() -> io.setVoltage(voltages[0], voltages[1]), io::stop))
                // Mark subsystem as zeroed
                .andThen(() -> zeroed = true);
    }

    public Command zeroInPlace() {
        return Commands.runOnce(() -> {
            io.resetPivotPosition(constants.restingPivotAngle());
            io.resetAmpPosition(constants.restingAmpAngle());
            zeroed = true;
        });
    }

    public record Constants(double minPivotAngle, double maxPivotAngle, double restingPivotAngle, double zeroPivotVoltage,
                            double minAmpAngle, double maxAmpAngle, double restingAmpAngle, double zeroAmpVoltage,
                            double pivotDangerAngle, double ampSafeAngle) {
    }
}
