package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.utility.SimUtil;

import static edu.wpi.first.math.util.Units.degreesToRadians;

/*
public class SimPivotIo implements PivotIo {
    private static final Pivot.Constants CONSTANTS = new Pivot.Constants(degreesToRadians(0.0), degreesToRadians(150.0), degreesToRadians(2.0), -1.0);;

    private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
    private static final double GEAR_RATIO = 50.0;
    private static final double CURRENT_LIMIT = 40.0;

    private final SingleJointedArmSim sim =
            new SingleJointedArmSim(MOTOR, GEAR_RATIO, 2.0, 0.3, CONSTANTS.minPivotAngle(), CONSTANTS.maxPivotAngle(), false, CONSTANTS.minPivotAngle());
    private final PIDController controller = new PIDController(10.0, 0.0, 0.0);

    private double targetAngle = Double.NaN;
    private double targetVoltage = 0.0;

    @Override
    public Pivot.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        double appliedVoltage = 0.0;
        if (Double.isFinite(targetAngle))
        {
            appliedVoltage = controller.calculate(sim.getAngleRads(), targetAngle);
        }
        else if (Double.isFinite(targetVoltage))
        {
            appliedVoltage = targetVoltage;
        }

        // Clamp voltage
        appliedVoltage = MathUtil.clamp(appliedVoltage, -12.0, 12.0);

        // Current limiting
        appliedVoltage = SimUtil.applyCurrentLimit(MOTOR, sim.getVelocityRadPerSec() * GEAR_RATIO, appliedVoltage, CURRENT_LIMIT);

        sim.setInputVoltage(appliedVoltage);
        sim.update(Robot.defaultPeriodSecs);

        inputs.pivotAngle = sim.getAngleRads();
        inputs.pivotAngularVelocity = sim.getVelocityRadPerSec();
        inputs.pivotAppliedVolts = appliedVoltage;
        inputs.pivotCurrentDrawAmps = Math.abs(MOTOR.getCurrent(inputs.pivotAngularVelocity * GEAR_RATIO, appliedVoltage));
    }

    @Override
    public void setTargetAngle(double pivotAngle) {
        if (Double.isNaN(targetAngle))
            controller.reset();
        targetAngle = pivotAngle;
        targetVoltage = Double.NaN;
    }

    @Override
    public void resetPosition(double pivotPosition) {
        sim.setState(pivotPosition, sim.getVelocityRadPerSec());
    }

    @Override
    public void setVoltage(double pivotVoltage) {
        targetAngle = Double.NaN;
        targetVoltage = pivotVoltage;
    }
}
*/