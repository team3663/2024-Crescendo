package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class Climber extends SubsystemBase {
    private static final double WAIT_TIME = 0.25;
    private static final double POSITION_THRESHOLD = inchesToMeters(1.0);
    private static final double VELOCITY_THRESHOLD = 0.01;
    private static final double HEIGHT_TOLERANCE = inchesToMeters(2);
    private double leftTargetHeight = 0.0;
    private double rightTargetHeight= 0.0;
    private final ClimberIo io;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();
    private final Constants constants;

    public Climber(ClimberIo io) {
        this.io = new LoggingClimberIo(io);
        this.constants = io.getConstants();
    }

    public Climber.Constants getConstants() {
        return constants;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public double getLeftHeight() {
        return inputs.leftPosition;
    }

    public double getRightHeight() {
        return inputs.rightPosition;
    }

    public boolean atTargetHeight() {
        return (Math.abs(leftTargetHeight - getLeftHeight()) < HEIGHT_TOLERANCE) &&
                (Math.abs(rightTargetHeight - getRightHeight()) < HEIGHT_TOLERANCE);
    }

    public Command follow(DoubleSupplier leftPositionSupplier, DoubleSupplier rightPositionSupplier) {
        return run(
                () -> {
                    leftTargetHeight = leftPositionSupplier.getAsDouble();
                    rightTargetHeight = rightPositionSupplier.getAsDouble();

                    io.setTargetPosition(leftTargetHeight, rightTargetHeight);
                }
        ).handleInterrupt(io::stop);
    }

    public Command follow(DoubleSupplier motorSupplier) {
        return follow(motorSupplier, motorSupplier);
    }

    public Command moveTo(double leftHeight, double rightHeight) {
        return
                follow(() -> leftHeight, () -> rightHeight)
                        .until(() -> Math.abs(inputs.leftPosition - leftHeight) < POSITION_THRESHOLD &&
                                Math.abs(inputs.rightPosition - rightHeight) < POSITION_THRESHOLD);
    }

    public Command moveTo(double motorHeight) {
        return moveTo(motorHeight, motorHeight);
    }

    public Command lock() {
        return runOnce(
                () -> io.setLocked(true, true));
    }

    public Command unlock() {
        return runOnce(
                () -> io.setLocked(false, false));
    }

    public Command zero() {
        // Wait until the climber stops moving
        return waitUntil(() -> Math.abs(inputs.leftVelocity) < VELOCITY_THRESHOLD
                && Math.abs(inputs.rightVelocity) < VELOCITY_THRESHOLD)
                // Then reset the climber position
                .andThen(io::resetPosition)
                // Before we check if we're at the bottom hard stop, wait a little
                .beforeStarting(waitSeconds(WAIT_TIME))
                // Retract while we haven't found the bottom hard stop
                .deadlineWith(runEnd(
                        () -> io.setVoltage(constants.armResetVoltage, constants.armResetVoltage),
                        io::stop))
                // Before we move, unlock the climber
                .beforeStarting(unlock())
                // After we finish, lock the climber
                .andThen(lock());
    }

    public record Constants(double maxArmHeight, double armResetVoltage) {
    }
}