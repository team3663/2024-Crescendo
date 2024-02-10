package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIo  {
    default void updateInputs(Inputs inputs) {}
    default void setVoltage(double voltage) {}

    default void setPrimaryMotorVoltage(double voltage) {}

    default void setSecondaryMotorVoltage(double voltage) {}

    class Inputs implements LoggableInputs {
        public double angularVelocity;
        public double appliedVolts;
        public double currentDrawAmps;
        public double motorTemp;
        public double beamBreakVoltage;

        @Override
        public void toLog(LogTable table) {
            table.put("AngularVelocity", angularVelocity);
            table.put("AppliedVolts", (appliedVolts));
            table.put("CurrentDrawAmps", (currentDrawAmps));
            table.put("MotorTemp", motorTemp);
        }

        @Override
        public void fromLog(LogTable table) {
            angularVelocity = table.get("AngularVelocity", 0.0);
            appliedVolts = table.get("AppliedVolts", 0.0);
            currentDrawAmps = table.get("CurrentDrawAmps", 0.0);
            motorTemp = table.get("MotorTemp", 0.0);
        }
    }
}
