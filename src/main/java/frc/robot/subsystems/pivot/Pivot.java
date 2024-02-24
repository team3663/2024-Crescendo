package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
    private static final double ANGLE_THRESHOLD = 0.5;
    public static final double VELOCITY_WAIT_SEC = 0.1;
    public static final double VELOCITY_ZERO_RANGE = 0.01;
    public static final double RESET_VOLTAGE = -2.0;

    private final PivotIo io;
    private final PivotInputsAutoLogged inputs = new PivotInputsAutoLogged();

    public Pivot(PivotIo io) {
        this.io = new LoggingPivotIo(io);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
    }

    public Command moveTo(double angle) {
        return run(() -> io.setTargetAngle(angle))
                .until(() -> Math.abs(angle - inputs.currentAngleRad) < ANGLE_THRESHOLD);
    }

    public Command zero() {
        return
                // Wait for pivot motor to start moving before determining whether motor velocity is zero
                Commands.waitSeconds(VELOCITY_WAIT_SEC)

                        // Checks whether the motor velocity is zero after wait period is over
                        .andThen(Commands.waitUntil(() -> Math.abs(inputs.currentVelocityRadPerSec) < VELOCITY_ZERO_RANGE)

                        // Moves at a set voltage and stops moving when motor velocity is zero/reaches the hard stop
                        .deadlineWith(runEnd(() -> io.setVoltage(RESET_VOLTAGE), io::stop)))

                        // Sensor position is reset to zero after pivot moves to its hard stop
                        .andThen(() -> io.resetPosition(0));
    }
}
