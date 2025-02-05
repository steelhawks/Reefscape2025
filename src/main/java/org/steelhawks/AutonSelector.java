package org.steelhawks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.util.VirtualSubsystem;

import java.util.List;

public class AutonSelector extends VirtualSubsystem {

    private static final AutoRoutine DEFAULT_ROUTINE =
        new AutoRoutine("Nothing auto", List.of(), Commands.print("No auto selected"));

    private String[] mQuestions = {
        "Starting Position",
        "First Placing Position",
        "Finishing Position"
    };

    @Override
    public void periodic() {

    }

    private record AutoRoutine(
        String name, List<AutoQuestion> questions, Command command) {}

    public record AutoQuestion(String question, List<AutoQuestionResponse> responses) {}

    /** Responses to auto routine questions. */
    public enum AutoQuestionResponse {
        L1, L2, // Left Reef
        BL1, BL2, // Back Left Reef
        BR1, BR2, // Back Right Reef
        R1, R2, // Right Reef
        TR1, TR2, // Top Right Reef
        TL1, TL2, // Top Left Reef
        CAGE1, // Climbing Cage 1
        CAGE2, // Climbing Cage 2
        CAGE3, // Climbing Cage 3
        PROCESSOR,
        YES,
        NO
    }

    private final LoggedDashboardChooser<AutoRoutine> autonChooser;

    public AutonSelector(String key) {
        autonChooser =
            new LoggedDashboardChooser<>(key + "/Routine");

        autonChooser.addDefaultOption("Nothing auto", DEFAULT_ROUTINE);
    }

    public Command getAutonCommand() {
        return Commands.none();
    }
}
