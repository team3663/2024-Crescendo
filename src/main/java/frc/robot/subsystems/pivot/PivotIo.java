package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface PivotIo {

    default void updateInputs(Inputs inputs) {}

    default void setTargetAngle(double rad) {}

    class Inputs implements LoggableInputs {
        public double inputVoltageRight = 0;
        public double inputVoltageLeft = 0;
        public double outputVoltageRight = 0;
        public double outputVoltageLeft = 0;
        public double currentAngleRad = 0;


        @Override
        public void toLog(LogTable table) {
            table.put("inputVoltageRight", inputVoltageRight);
            table.put("inputVoltageLeft", inputVoltageLeft);
            table.put("outputVoltageRight", outputVoltageRight);
            table.put("outputVoltageLeft", outputVoltageLeft);
            table.put("currentAngleRad", currentAngleRad);
        }

        @Override
        public void fromLog(LogTable table) {
            inputVoltageRight = table.get("inputVoltageRight", inputVoltageRight);
            inputVoltageLeft = table.get("inputVoltageLeft", inputVoltageLeft);
            outputVoltageRight = table.get("outputVoltageRight", outputVoltageRight);
            outputVoltageLeft = table.get("outputVoltageLeft", outputVoltageLeft);
            currentAngleRad = table.get("currentAngleRad", currentAngleRad);
        }
    }
}
