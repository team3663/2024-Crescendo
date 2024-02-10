package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class Climber extends SubsystemBase {
    private final ClimberIo io;
    private static final double waitTime = 0.1;
    private static final double velocityThreshold = 0.01;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();
    private final Constants constants;

    public Climber(ClimberIo io) {
        this.io = io;
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

    public Command follow(DoubleSupplier leftPositionSupplier, DoubleSupplier rightPositionSupplier) {
        return run(
                () -> io.setTargetPosition(leftPositionSupplier.getAsDouble(), rightPositionSupplier.getAsDouble())
        );
    }

    public Command follow(DoubleSupplier motorSupplier) {
        return follow(motorSupplier, motorSupplier);
    }

    public Command moveTo(double leftHeight, double rightHeight) {
        return follow(() -> leftHeight, () -> rightHeight);
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
        return waitSeconds(waitTime)
                .andThen(waitUntil(
                        () -> Math.abs(inputs.leftCurrentVelocity) < velocityThreshold
                                && Math.abs(inputs.rightCurrentVelocity) < velocityThreshold))
                .deadlineWith(runEnd(
                        () -> io.setVoltage(constants.armResetVoltage, constants.armResetVoltage),
                        io::stop))
                .andThen(io::resetPosition);
    }

    public record Constants(double maxArmHeight, double armResetVoltage) {
    }
}