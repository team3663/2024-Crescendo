package frc.robot.utility;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.led.LedColor;

import java.util.Map;

public class RobotMode {

    public enum ScoreLocation {
        SPEAKER,
        AMP,
        TRAP
    }

    private final LedColor red = new LedColor(255,0,0);
    private final LedColor yellow = new LedColor(255,255, 0);
    private final LedColor blue = new LedColor(0,0,255);
    private final LedColor black = new LedColor(0, 0, 0);

    private static GenericEntry scoreSpeakerEntry;
    private static GenericEntry scoreTrapEntry;
    private static GenericEntry scoreAmpEntry;

    private static ScoreLocation scoreLocation;

    public RobotMode() {
        // Default to scoring in the speaker
        scoreLocation = ScoreLocation.SPEAKER;
    }

    public void setScoreLocation(ScoreLocation location) {
        scoreLocation = location;

        scoreSpeakerEntry.setBoolean(scoreLocation == ScoreLocation.SPEAKER);
        scoreTrapEntry.setBoolean(scoreLocation == ScoreLocation.TRAP);
        scoreAmpEntry.setBoolean(scoreLocation == ScoreLocation.AMP);
    }

    public static ScoreLocation getScoreLocation() {
        return scoreLocation;
    }

    public LedColor getLedColor(ScoreLocation mode) {
        switch(mode) {
            case SPEAKER:
                return blue;
            case AMP:
                return yellow;
            case TRAP:
                return red;
            default:
                return black;
        }
    }

    private static void SetupShuffleboard()
    {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver");

        scoreSpeakerEntry = tab.add("Speaker", scoreLocation == ScoreLocation.SPEAKER)
                .withPosition(4, 0)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
                .getEntry();

        scoreTrapEntry = tab.add("Trap", scoreLocation == ScoreLocation.TRAP)
                .withPosition(4, 1)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
                .getEntry();

        scoreAmpEntry = tab.add("Amp", scoreLocation == ScoreLocation.AMP)
                .withPosition(4, 2)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
                .getEntry();
    }
}
