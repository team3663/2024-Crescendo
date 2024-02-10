package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
   private final PivotIo io;
   private final PivotIo.Inputs inputs = new PivotIo.Inputs();

    public Pivot(PivotIo io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
    }

    public Command setTargetAngle(double angle) {
        Logger.recordOutput("TargetAngleRad", angle);
        return run(
                () -> io.setTargetAngle(angle)
        ).until(
                () -> Math.abs(angle - inputs.currentAngleRad) < 0.5
        );
    }
}
