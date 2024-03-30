package frc.robot.config;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.climber.ClimberIo;
import frc.robot.subsystems.climber.SimClimberIo;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.SimpleSimDrivetrain;
import frc.robot.subsystems.feeder.C2024FeederIo;
import frc.robot.subsystems.feeder.FeederIo;
import frc.robot.subsystems.shooter.ShooterIo;
import frc.robot.subsystems.shooter.SimShooterIo;

public class SimRobotFactory implements RobotFactory {
    @Override
    public ClimberIo createClimberIo() {
        return new SimClimberIo();
    }

    @Override
    public DrivetrainIO createDrivetrainIO() {
        return new SimpleSimDrivetrain();
    }

    @Override
    public FeederIo createFeederIo() {
        return new C2024FeederIo(
                new TalonFX(1),
                new DigitalInput(0)
        );
    }

//    @Override
//    public PivotIo createPivotIo() {
//        return new SimPivotIo();
//    }

    @Override
    public ShooterIo createShooterIo() {
        return new SimShooterIo();
    }
}
