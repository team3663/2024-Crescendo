package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIo {
    default void updateInputs(IntakeInputs inputs) {
    }

    default void setRollerVoltage(double voltage) {
    }

    default void setCenteringVoltage(double voltage) {
    }

    @AutoLog
    class IntakeInputs {
        public double rollerAngularVelocity;
        public double rollerAppliedVolts;
        public double rollerCurrentDrawAmps;
        public double rollerMotorTemp;
        public double centeringAngularVelocity;
        public double centeringAppliedVolts;
        public double centeringCurrentDrawAmps;
        public double centeringMotorTemp;
    }
}