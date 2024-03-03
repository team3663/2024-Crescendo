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
        
        public double leftCenteringAngularVelocity;
        public double leftCenteringAppliedVolts;
        public double leftCenteringCurrentDrawAmps;
        public double leftCenteringMotorTemp;
        
        public double rightCenteringAngularVelocity;
        public double rightCenteringAppliedVolts;
        public double rightCenteringCurrentDrawAmps;
        public double rightCenteringMotorTemp;
    }
}