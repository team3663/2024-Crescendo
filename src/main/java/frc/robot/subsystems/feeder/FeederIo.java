package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.StatusSignal;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FeederIo {
    default void updateInputs(FeederInputs inputs) {}
    default void setVoltage(double voltage)  {}

    @AutoLog
    class FeederInputs {
        public double angularVelocity;
        public double appliedVolts;
        public double currentDrawAmps;
        public double motorTemp;
        public double beamBreakVoltage;
        public boolean beamBreakSignaled;
    }
}