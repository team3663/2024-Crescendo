package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
    private final DrivetrainIO io;
    private final DrivetrainIO.Inputs inputs = new DrivetrainIO.Inputs();

    public Drivetrain(DrivetrainIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain", inputs);
    }

    public Command drive(
            DoubleSupplier xVelocity,
            DoubleSupplier yVelocity,
            DoubleSupplier rotationalVelocity
    ) {
        return runEnd(
                // execute()
                () -> io.driveFieldOriented(
                        xVelocity.getAsDouble(),
                        yVelocity.getAsDouble(),
                        rotationalVelocity.getAsDouble()
                ),
                // end()
                io::stop
        );
    }

    public Command zeroGyroscope() {
        return Commands.runOnce(io::zeroGyroscope);
    }
}
