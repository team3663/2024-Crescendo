package frc.robot.subsystems.climber;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.math.util.Units.feetToMeters;
import static edu.wpi.first.math.util.Units.inchesToMeters;

public interface ClimberIo {
    default Climber.Constants getConstants() {
        return new Climber.Constants(0,-2.0);
    }

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
        public double leftCurrentVelocity;
        public double leftMotorTemp;
        public boolean leftLocked;
        public double leftVelocity;
        public double rightPosition;
        public double rightAppliedVolts;
        public double rightCurrentDrawAmps;
        public double rightCurrentVelocity;

        public double rightMotorTemp;
        public boolean rightLocked;
        public double rightVelocity;

    }
}
