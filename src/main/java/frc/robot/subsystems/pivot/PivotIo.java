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
        public double currentAngleRadRight = 0;
        public double currentAngleRadLeft = 0;
        public double targetAngleRad = 0;


        @Override
        public void toLog(LogTable table) {
            table.put("inputVoltageRight", inputVoltageRight);
            table.put("inputVoltageLeft", inputVoltageLeft);
            table.put("outputVoltageRight", outputVoltageRight);
            table.put("outputVoltageLeft", outputVoltageLeft);
            table.put("currentAngleRadRight", currentAngleRadRight);
            table.put("currentAngleRadLeft", currentAngleRadLeft);
            table.put("targetAngleRad", targetAngleRad);

        }

        @Override
        public void fromLog(LogTable table) {
            inputVoltageRight = table.get("inputVoltageRight", inputVoltageRight);
            inputVoltageLeft = table.get("inputVoltageLeft", inputVoltageLeft);
            outputVoltageRight = table.get("outputVoltageRight", outputVoltageRight);
            outputVoltageLeft = table.get("outputVoltageLeft", outputVoltageLeft);
            currentAngleRadRight = table.get("currentAngleRadRight", currentAngleRadRight);
            currentAngleRadLeft = table.get("currentAngleRadLeft", currentAngleRadLeft);
            targetAngleRad = table.get("targetAngleRad", targetAngleRad);
        }
    }
}
