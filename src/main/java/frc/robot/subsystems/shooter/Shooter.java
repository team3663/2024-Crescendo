package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(100);
    private final ShooterIo io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    public Shooter(ShooterIo io) {
        this.io = new LoggingShooterIo(io);
    }
    private double targetVelocity = 0.0;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    /**
     * Return whether the shooter velocity is within a specified percent of the target velocity.
     * @param thresholdPercent - Percent of target velocity that shooter must be running at to be considered at speed.
     *
     * @return True if shooter velocity is within threshold% of the target velocity.
     */
    public boolean atTargetVelocity()
    {
        return Math.abs(targetVelocity - inputs.lowerAngularVelocity) < VELOCITY_THRESHOLD;
    }

    /**
     * Create a command to run the shooter at a specified velocity
     * @param velocity - Velocity to run shooter at (radians/second)
     * @return - Command
     */
    public Command setTargetVelocity(double velocity) {
        return startEnd(
                () -> { targetVelocity = velocity; io.setTargetVelocity(velocity); },
                () -> { targetVelocity = 0; io.stop(); }
        );
    }

    public Command runWithVoltage(double voltage) {
        return startEnd(() -> io.setVoltage(voltage), io::stop);
    }

    public Command stop() {
        return runOnce(() -> {targetVelocity = 0; io.stop();});
    }
}
