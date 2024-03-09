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
        return inputs.angle;
    }

    public boolean isZeroed() {
        return zeroed;
    }

    public Command follow(DoubleSupplier angleSupplier) {
        return run(() -> {
            double angle = angleSupplier.getAsDouble();

            // Do nothing if not zeroed
            if (!zeroed) return;

            // Ignore if target angle is not finite
            if (!Double.isFinite(angle)) return;

            io.setTargetAngle(MathUtil.clamp(angle, constants.minAngle(), constants.maxAngle()));
        });
    }

    public Command moveTo(double angle) {
        return follow(() -> angle)
                .until(() -> Math.abs(angle - inputs.angle) < ANGLE_THRESHOLD);
    }

    public Command zero() {
        return
                // Wait for pivot motor to start moving before determining whether motor velocity is zero
                Commands.waitSeconds(VELOCITY_WAIT_SEC)

                        // Checks whether the motor velocity is zero after wait period is over
                        .andThen(Commands.waitUntil(() -> Math.abs(inputs.angularVelocity) < VELOCITY_ZERO_RANGE))

                        // Moves at a set voltage and stops moving when motor velocity is zero/reaches the hard stop
                        .deadlineWith(runEnd(() -> io.setVoltage(constants.zeroVoltage()), io::stop))

                        // Sensor position is reset to zero after pivot moves to its hard stop
                        .andThen(() -> {
                            io.resetPosition(0);

                            zeroed = true;
                        });
    }

    public Command zeroInPlace() {
        return Commands.runOnce(() -> {
            io.resetPosition(constants.restingAngle());
            zeroed = true;
        });
    }

    public record Constants(double minAngle, double maxAngle, double restingAngle, double zeroVoltage) {
    }
}
