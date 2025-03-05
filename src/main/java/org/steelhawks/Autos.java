package org.steelhawks;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.util.autonbuilder.AutonBuilder;
import org.steelhawks.util.autonbuilder.StartEndPosition;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import java.io.IOException;
import java.util.ArrayList;

public final class Autos {
    private static final AutonBuilder s_Builder = new AutonBuilder("Auto Selector");

    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Intake s_Intake = RobotContainer.s_Intake;

    private static final LoggedDashboardChooser<Command> autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser");

    public static void init() {
        autoChooser.addDefaultOption("Nothing Auto", Commands.none().withName("Nothing Auto"));
        autoChooser.addOption("Use Auton Builder", Commands.none().withName("Use Auton Builder"));
        autoChooser.addOption("BC1 Auton", getBC1Auton());
        autoChooser.addOption("BC2 Auton", getBC2Auton());
        autoChooser.addOption("BC3 Auton", getBC3Auton());
        autoChooser.addOption("RC1 Auton", getRC1Auton());
        autoChooser.addOption("RC2 Auton", getRC2Auton());
        autoChooser.addOption("RC3 Auton", getRC3Auton());
    }

    public static Command followTrajectory(String choreo) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(choreo);
            return DriveCommands.followPath(path).withName("Following " + choreo);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public static Command elevatorAndShoot(ElevatorConstants.State state) {
        return Commands.sequence(
            s_Elevator.setDesiredState(state),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.waitUntil(s_Elevator.atThisGoal(state))),
            s_Intake.shootCoral().withTimeout(1.0),
            s_Elevator.noSlamCommand())
            .withName("Elevator and Shoot in Auton");
    }

    private static boolean endsWithSource(String trajectory) {
        String[] words = trajectory.split(" ");
        return words[words.length - 1].equals("Source");
    }

    private static Command buildTrajectorySequence(String[] trajectories) {
        ArrayList<Command> commands = new ArrayList<>();

        for (String trajectory : trajectories) {
            commands.add(followTrajectory(trajectory));
            if (endsWithSource(trajectory)) {
//                commands.add(Commands.waitUntil(s_Intake.hasCoral()));
                commands.add(Commands.waitSeconds(1));
            } else {
                commands.add(elevatorAndShoot(ElevatorConstants.State.L4));
            }
        }

        return Commands.sequence(commands.toArray(new Command[commands.size()]));
    }

    private static Command createAuto(StartEndPosition pose, String[] trajectories) {
        return Commands.runOnce(() -> s_Swerve.setPose(AllianceFlip.apply(pose.getPose())))
            .andThen(buildTrajectorySequence(trajectories));
    }

    public static Command getBC1AutonTest() {
        return Commands.runOnce(
            () -> s_Swerve.setPose(AllianceFlip.apply(StartEndPosition.BC1.getPose())))
            .andThen(
                followTrajectory("BC1 to TR2"),
                s_Elevator.setDesiredState(ElevatorConstants.State.L4),
                Commands.race(
                    Commands.waitSeconds(1),
                    Commands.waitUntil(s_Elevator.atGoal())),
                s_Intake.shootCoralSlow().withTimeout(1.0),
                s_Elevator.setDesiredState(ElevatorConstants.State.HOME))
            .andThen(followTrajectory("TR2 to Upper Source"));
    }

    public static Command getTestPath() {
        return Commands.runOnce(
                        () -> s_Swerve.setPose(AllianceFlip.apply(StartEndPosition.BC1.getPose())))
                .andThen(
                        followTrajectory("Test Path"));
    }

    public static Command getBC1Auton() {
        return createAuto(StartEndPosition.BC1,
            new String[]{
                "BC1 to TL1",
                "TL1 to Upper Source",
                "Upper Source to TL2",
                "TL2 to Upper Source",
                "Upper Source to L1",
                "L1 to Upper Source",
                "Upper Source to L1"
            }).withName("BC1 Auto");
    }

    public static Command getBC2Auton() {
        return createAuto(StartEndPosition.BC2,
            new String[]{
                "BC2 to TR1",
                "TR1 to Upper Source",
                "Upper Source to TR2",
                "TR2 to Upper Source",
                "Upper Source to L1",
                "L1 to Upper Source",
                "Upper Source to L1"
            }).withName("BC2 Auto");
    }

    public static Command getBC3Auton() {
        return createAuto(StartEndPosition.BC3,
            new String[]{
                "BC3 to TR1",
                "TR1 to Upper Source",
                "Upper Source to TR2",
                "TR2 to Upper Source",
                "Upper Source to L1",
                "L1 to Upper Source",
                "Upper Source to L1"
            }).withName("BC3 Auto");
    }

    public static Command getRC1Auton() {
        return createAuto(StartEndPosition.RC1,
            new String[]{
                "RC1 to BR1",
                "BR1 to Lower Source",
                "Lower Source to BL1",
                "BL1 to Lower Source",
                "Lower Source to BL1",
                "BL1 to Lower Source",
                "Lower Source to BL2"
            }).withName("RC1 Auto");
    }

    public static Command getRC2Auton() {
        return createAuto(StartEndPosition.RC2,
            new String[]{
                "RC2 to BR2"
                    // "BR2 to Lower Source",
                    // "Lower Source to BR1"
//                "BR1 to Lower Source",
//                "Lower Source to BL1",
//                "BL1 to Lower Source",
//                "Lower Source to BL1",
//                "BL1 to Lower Source",
//                "Lower Source to BL2"
            }).withName("RC2 Auto");
    }

    public static Command getRC3Auton() {
        return createAuto(StartEndPosition.RC3,
            new String[]{
                "RC3 to BR1",
                "BR1 to Lower Source",
                "Lower Source to BL1",
                "BL1 to Lower Source",
                "Lower Source to BL1",
                "BL1 to Lower Source",
                "Lower Source to BL2"
            }).withName("RC3 Auto");
    }

    public static Command getAuto() {
        if (autoChooser.get().getName().equals("Use Auton Builder")) {
            return s_Builder.getAutonCommand();
        }
        return autoChooser.get();
    }
}
