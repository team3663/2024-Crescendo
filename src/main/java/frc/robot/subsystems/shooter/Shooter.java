package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIo io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    public Shooter(ShooterIo io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public double getVelocity() {
        return inputs.upperAngularVelocity;
    }

    public Command setTargetVelocity(double velocity) {
        return run(
                () -> io.setTargetVelocity(velocity)
        );
    }

    public Command runWithVoltage(double voltage) {
        return runEnd(
                () -> {io.setShooterVoltage(voltage);},
                () -> {io.setShooterVoltage(0.0);}
        );
    }

    public Command stop() {
        return runOnce(io::stop);
    }
}
