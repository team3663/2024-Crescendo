// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final double DRIVETRAIN_TRACKWIDTH = inchesToMeters(28.0);
    public static final double DRIVETRAIN_WHEELBASE = inchesToMeters(30.0);
    public static final String DRIVETRAIN_CANBUS_NAME = "3663";

    public static final int DRIVETRAIN_PIGEON2_ID = 13;

    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int DRIVETRAIN_FRONT_LEFT_STEER_MOTOR_ID = 5;
    public static final int DRIVETRAIN_FRONT_LEFT_STEER_ENCODER_ID = 9;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
    public static final int DRIVETRAIN_FRONT_RIGHT_STEER_MOTOR_ID = 6;
    public static final int DRIVETRAIN_FRONT_RIGHT_STEER_ENCODER_ID = 10;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR_ID = 3;
    public static final int DRIVETRAIN_BACK_LEFT_STEER_MOTOR_ID = 7;
    public static final int DRIVETRAIN_BACK_LEFT_STEER_ENCODER_ID = 11;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR_ID = 8;
    public static final int DRIVETRAIN_BACK_RIGHT_STEER_MOTOR_ID = 4;
    public static final int DRIVETRAIN_BACK_RIGHT_STEER_ENCODER_ID = 12;

    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = degreesToRadians(116.63);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = degreesToRadians(-86.04);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = degreesToRadians(-202.0);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = degreesToRadians(-279.67);
}
