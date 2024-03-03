package frc.robot.config;

import com.ctre.phoenix.led.CANdle;
import frc.robot.subsystems.climber.ClimberIo;
import frc.robot.subsystems.climber.SimClimberIo;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.SimpleSimDrivetrain;
import frc.robot.subsystems.led.LedCandleIo;
import frc.robot.subsystems.led.LedIo;
import frc.robot.subsystems.shooter.ShooterIo;
import frc.robot.subsystems.shooter.SimShooterIo;

public class BenchRobotFactory implements RobotFactory {
/*    @Override
    public LedIo createLedIo() {
        CANdle candle = new CANdle(1);
        return new LedCandleIo(candle);
    }*/
}
