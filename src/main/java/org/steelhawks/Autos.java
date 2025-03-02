package org.steelhawks;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.json.simple.parser.ParseException;
import org.steelhawks.autonselector.StartEndPosition;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;

import java.io.IOException;
import java.util.ArrayList;

public final class Autos {

    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Intake s_Intake = RobotContainer.s_Intake;

    private static final AutoFactory autoFactory =
        new AutoFactory(
            s_Swerve::getPose,
            s_Swerve::setPose,
            s_Swerve::followChoreoTrajectory,
            true,
            s_Swerve);

    public static Command followTrajectory(String choreo) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(choreo);
            return DriveCommands.followPath(path);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public static AutoRoutine testChoreoAuton() {
        AutoRoutine routine = autoFactory.newRoutine("Testing");

        AutoTrajectory testPath = routine.trajectory("test path");

        routine.active().onTrue(
            testPath.resetOdometry()
            .andThen(testPath.cmd()));

        return routine;
    }

    public static AutoRoutine testAuton() {
        AutoRoutine testAuton = autoFactory.newRoutine("test");
        AutoTrajectory BC3_TO_R1 = testAuton.trajectory("BC3 to R1");

        testAuton.active().onTrue(
            Commands.sequence(
                BC3_TO_R1.resetOdometry(), 
                BC3_TO_R1.cmd()));

        return testAuton;
    }

    public static Command elevatorAndShoot(ElevatorConstants.State state) {
        return Commands.sequence(
            s_Elevator.setDesiredState(state),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.waitUntil(s_Elevator.atThisGoal(state))),
            s_Intake.shootCoral().withTimeout(1.0),
            s_Elevator.noSlamCommand());
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
                commands.add(Commands.waitUntil(s_Intake.hasCoral()));
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

    public static Command getPathPlannerAuton() {
        return new PathPlannerAuto("Experimental Auto");
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
            });
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
            });
    }

    public static Command getBC3Auton() {
        return createAuto(StartEndPosition.BC2,
            new String[]{
                "BC3 to TR1",
                "TR1 to Upper Source",
                "Upper Source to TR2",
                "TR2 to Upper Source",
                "Upper Source to L1",
                "L1 to Upper Source",
                "Upper Source to L1"
            });
    }

    public static Command getRC1Auton() {
        return createAuto(StartEndPosition.BC2,
            new String[]{
                "RC1 to BR1",
                "BR1 to Lower Source",
                "Lower Source to BL1",
                "BL1 to Lower Source",
                "Lower Source to BL1",
                "BL1 to Lower Source",
                "Lower Source to BL2"
            });
    }

    public static Command getRC2Auton() {
        return createAuto(StartEndPosition.BC2,
            new String[]{
                "RC2 to BR1",
                "BR1 to Lower Source",
                "Lower Source to BL1",
                "BL1 to Lower Source",
                "Lower Source to BL1",
                "BL1 to Lower Source",
                "Lower Source to BL2"
            });
    }

    public static Command getRC3Auton() {
        return createAuto(StartEndPosition.BC2,
            new String[]{
                "RC3 to BR1",
                "BR1 to Lower Source",
                "Lower Source to BL1",
                "BL1 to Lower Source",
                "Lower Source to BL1",
                "BL1 to Lower Source",
                "Lower Source to BL2"
            });
    }
}
