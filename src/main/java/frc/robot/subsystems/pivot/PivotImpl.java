package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.PIDController;

public class PivotImpl implements PivotIO{

    private final PIDController pid = new PIDController(5,0,0);

    private double angleRad = 0;

    private double voltage = 0;

    private double radRotRatio = 1;

    public PivotImpl(double radRotRatio) {
        this.radRotRatio = radRotRatio;
    }

    public void updateInputs(Inputs inputs) {
        inputs.angleRad = angleRad;
        inputs.voltage = voltage;
    }

    private double radToRot(double angleRad) {
        return angleRad / radRotRatio;
    }

    private double rotToRad(double rot) {
        return rot * radRotRatio;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setTargetAngle(double angleRad) {

    }
}
