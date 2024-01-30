package frc.robot.subsystems.Climber;

import frc.robot.subsystems.intake.IntakeIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ClimberIO {

    default void updateInputs(ClimberIO.Inputs inputs) {}
    void stop();
    void resetPosition();
    void setTargetPosition();
    void setVoltage(double voltageLeft,double voltageRight);
    void setLocked(boolean lockedLeft, boolean lockedRight);

    class Inputs implements LoggableInputs {
        public double position;
        public double appliedVolts;
        public double currentDrawAmps;
        public double motorTemp;
        public boolean isLocked;

        @Override
        public void toLog(LogTable table) {
            table.put("Position", position);
            table.put("AppliedVolts", appliedVolts);
            table.put("CurrentDrawAmps", currentDrawAmps);
            table.put("MotorTemp", motorTemp);
            table.put("Locked:", isLocked);
        }

        @Override
        public void fromLog(LogTable table) {
            position = table.get("Position", 0.0);
            appliedVolts = table.get("AppliedVolts", 0.0);
            currentDrawAmps = table.get("CurrentDrawAmps", 0.0);
            motorTemp = table.get("MotorTemp", 0.0);
            isLocked = table.get("Locked:", false);
        }


    }
}
