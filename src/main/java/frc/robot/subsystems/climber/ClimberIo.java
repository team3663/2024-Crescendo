package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ClimberIo {

    default void updateInputs(ClimberInputs inputs) {}
    void stop();
    void resetPosition();
    void setTargetPosition( double leftHeight, double rightHeight);
    void setVoltage(double voltageLeft,double voltageRight);
    void setLocked(boolean lockedLeft, boolean lockedRight);
    @AutoLog
    class ClimberInputs{
        public double leftPosition;
        public double leftAppliedVolts;
        public double leftCurrentDrawAmps;
        public double leftMotorTemp;
        public boolean leftLocked;
        public double leftVelocity;
        public double rightPosition;
        public double rightAppliedVolts;
        public double rightCurrentDrawAmps;
        public double rightMotorTemp;
        public boolean rightLocked;
        public double rightVelocity;

    }
}
