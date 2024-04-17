package frc.robot.utility;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Map;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class RobotMode {

    public enum ScoreLocation {
        SPEAKER,
        AMP,
        SUBWOOFER,
        PASS
    }

    private static GenericEntry scoreSpeakerEntry;
    private static GenericEntry scoreSubwooferEntry;
    private static GenericEntry scoreAmpEntry;
    private static GenericEntry scorePassEntry;
    private static ScoreLocation scoreLocation;

    static {
        setupShuffleboard();
        setScoreLocation(ScoreLocation.SPEAKER);
    }
    public static ScoreLocation getScoreLocation() {
        return scoreLocation;
    }

    public static void setScoreLocation(ScoreLocation location) {
        scoreLocation = location;

        scoreSpeakerEntry.setBoolean(scoreLocation == ScoreLocation.SPEAKER);
        scoreSubwooferEntry.setBoolean(scoreLocation == ScoreLocation.SUBWOOFER);
        scoreAmpEntry.setBoolean(scoreLocation == ScoreLocation.AMP);
        scorePassEntry.setBoolean(scoreLocation == ScoreLocation.PASS);
    }

    private static void setupShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver");

        scoreAmpEntry = tab.add("Amp", scoreLocation == ScoreLocation.AMP)
                .withPosition(4, 0)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "#0000FF", "Color when false", "#FFFFFF"))
                .getEntry();

        scoreSpeakerEntry = tab.add("Speaker", scoreLocation == ScoreLocation.SPEAKER)
                .withPosition(5, 0)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "#F6BE00", "Color when false", "#FFFFFF"))
                .getEntry();

        scoreSubwooferEntry = tab.add("Subwoofer", scoreLocation == ScoreLocation.SUBWOOFER)
                .withPosition(6, 0)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#FFFFFF"))
                .getEntry();

        scorePassEntry = tab.add("Pass", scoreLocation == ScoreLocation.PASS)
                .withPosition(7, 0)
                .withSize(1,1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
                .getEntry();
    }

    public static Command scoreLocation(ScoreLocation location) {
        return runOnce(
                () -> setScoreLocation(location)
        );
    }
}
