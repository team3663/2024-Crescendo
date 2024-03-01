package frc.robot.config;

import frc.robot.subsystems.climber.ClimberIo;
import frc.robot.subsystems.climber.SimClimberIo;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.SimpleSimDrivetrain;
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
    public ShooterIo createShooterIo() {
        return new SimShooterIo();
    }
}
