package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface PivotIO {

    default void updateInputs(Inputs inputs) {}

    default void setTargetAngle(double rad) {}

    class Inputs implements LoggableInputs {
        public double voltage = 0;
        public double angleRad = 0;

        @Override
        public void toLog(LogTable table) {
            table.put("Voltage", voltage);
            table.put("AngleRad", angleRad);
        }

        @Override
        public void fromLog(LogTable table) {
            voltage = table.get("Voltage", voltage);
            angleRad = table.get("AngleRad", angleRad);
        }
    }
}
