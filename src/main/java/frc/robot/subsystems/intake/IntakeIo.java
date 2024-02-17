package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIo {
    default void updateInputs(Inputs inputs) {}
    default void setIntakeVoltage(double voltage) {}
    default void setCenteringVoltage(double voltage) {}


    class Inputs implements LoggableInputs {
        public double intakeAngularVelocity;
        public double intakeAppliedVolts;
        public double intakeCurrentDrawAmps;
        public double intakeMotorTemp;
        public double centeringAngularVelocity;
        public double centeringAppliedVolts;
        public double centeringCurrentDrawAmps;
        public double centeringMotorTemp;


        @Override
        public void toLog(LogTable table) {
            table.put("IntakeAngularVelocity", intakeAngularVelocity);
            table.put("IntakeAppliedVolts", intakeAppliedVolts);
            table.put("IntakeCurrentDrawAmps", intakeCurrentDrawAmps);
            table.put("IntakeMotorTemp", intakeMotorTemp);

            table.put("CenteringAngularVelocity", centeringAngularVelocity);
            table.put("CenteringAppliedVolts", centeringAppliedVolts);
            table.put("CenteringCurrentDrawAmps", centeringCurrentDrawAmps);
            table.put("CenteringMotorTemp", centeringMotorTemp);
        }

        @Override
        public void fromLog(LogTable table) {
            intakeAngularVelocity = table.get("IntakeAngularVelocity", 0.0);
            intakeAppliedVolts = table.get("IntakeAppliedVolts", 0.0);
            intakeCurrentDrawAmps = table.get("IntakeCurrentDrawAmps", 0.0);
            intakeMotorTemp = table.get("IntakeMotorTemp", 0.0);

            centeringAngularVelocity = table.get("CenteringAngularVelocity", 0.0);
            centeringAppliedVolts = table.get("CenteringAppliedVolts", 0.0);
            centeringCurrentDrawAmps = table.get("CenteringCurrentDrawAmps", 0.0);
            centeringMotorTemp = table.get("CenteringMotorTemp", 0.0);
        }
    }
}