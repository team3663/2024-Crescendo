package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIo  {
    default void updateInputs(Inputs inputs) {}
    default void setTargetVelocity(double voltage) {}

    class Inputs implements LoggableInputs {
        public double upperAngularVelocity;
        public double upperAppliedVolts;
        public double upperCurrentDrawAmps;
        public double upperMotorTemp;

        public double lowerAngularVelocity;
        public double lowerAppliedVolts;
        public double lowerCurrentDrawAmps;
        public double lowerMotorTemp;

        @Override
        public void toLog(LogTable table) {
            table.put("UpperAngularVelocity", upperAngularVelocity);
            table.put("UpperAppliedVolts", (upperAppliedVolts));
            table.put("UpperCurrentDrawAmps", (upperCurrentDrawAmps));
            table.put("UpperMotorTemp", upperMotorTemp);


            table.put("LowerAngularVelocity", lowerAngularVelocity);
            table.put("LowerAppliedVolts", (lowerAppliedVolts));
            table.put("LowerCurrentDrawAmps", (lowerCurrentDrawAmps));
            table.put("LowerMotorTemp", lowerMotorTemp);
        }

        @Override
        public void fromLog(LogTable table) {
            upperAngularVelocity = table.get("UpperAngularVelocity", 0.0);
            upperAppliedVolts = table.get("UpperAppliedVolts", 0.0);
            upperCurrentDrawAmps = table.get("UpperCurrentDrawAmps", 0.0);
            upperMotorTemp = table.get("UpperMotorTemp", 0.0);

            lowerAngularVelocity = table.get("LowerAngularVelocity", 0.0);
            lowerAppliedVolts = table.get("LowerAppliedVolts", 0.0);
            lowerCurrentDrawAmps = table.get("LowerCurrentDrawAmps", 0.0);
            lowerMotorTemp = table.get("LowerMotorTemp", 0.0);
        }
    }
}
