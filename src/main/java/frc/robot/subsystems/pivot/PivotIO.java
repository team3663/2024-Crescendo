package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface PivotIO {

    default void updateInputs(Inputs inputs) {}

    default void setTargetAngle(double rad) {}

    class Inputs implements LoggableInputs {
        public double inputVoltage = 0;
        public double outputVoltage = 0;
        public double targetAngleRad = 0;
        public double currentAngleRad = 0;


        @Override
        public void toLog(LogTable table) {
            table.put("inputVoltage", inputVoltage);
            table.put("outputVoltage", outputVoltage);
            table.put("targetAngleRad", targetAngleRad);
            table.put("currentAngleRad", currentAngleRad);
        }

        @Override
        public void fromLog(LogTable table) {
            inputVoltage = table.get("inputVoltage", inputVoltage);
            outputVoltage = table.get("outputVoltage", outputVoltage);
            targetAngleRad = table.get("targetAngleRad", targetAngleRad);
            currentAngleRad = table.get("currentAngleRad", currentAngleRad);
        }
    }
}
