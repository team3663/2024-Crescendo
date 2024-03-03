package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

public class ControllerHelper {

    private static final double deadbandWidth = 0.1;

    /**
     * @param value - Raw value read from controller axis to be modified
     * @return clipped and scaled axis value to use.
     */
    public static double modifyAxis(double value) {
        double clippedValue = MathUtil.applyDeadband(value, deadbandWidth);

        // Square the clipped value (preserving the sign) and return it.
        return Math.copySign(clippedValue * clippedValue, value);
    }

    /**
     * Create command to enable rumble feedback on XBox controller when a condition is true.
     * @param controller - XBox controller to rumble
     * @param condition - Condition that triggers rumble.
     * @return Command that triggers rumble feedback on controller when condition is true.
     */
    public static Command rumble(CommandXboxController controller, BooleanSupplier condition) {

        return runEnd(
                () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, condition.getAsBoolean() ? 1.0 : 0.0),
                () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        );
    }
}
