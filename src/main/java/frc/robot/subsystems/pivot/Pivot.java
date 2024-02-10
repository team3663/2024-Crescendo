package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
    private static final double ANGLE_THRESHOLD = 0.5;

    private final PivotIo io;
    private final PivotInputsAutoLogged inputs = new PivotInputsAutoLogged();

    public Pivot(PivotIo io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
    }

    public Command moveTo(double angle) {
        return run(() -> {
            Logger.recordOutput("Pivot/TargetAngleRad", angle);

            io.setTargetAngle(angle);
        }).until(() -> Math.abs(angle - inputs.currentAngleRad) < ANGLE_THRESHOLD);
    }
}
