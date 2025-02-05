package org.steelhawks;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.json.simple.parser.ParseException;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.swerve.Swerve;

import java.io.IOException;

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

    private static final Alert noAutosSelectedAlert = new Alert("No auton selected", AlertType.kError);

    private static Command followTrajectory(String choreo) {
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

    public static Command getAutonCommand() {
        return Commands.none();
    }

    public static Command getTestAuton() {
        return Commands.runOnce(
            () ->
                RobotContainer.s_Swerve.setPose(
                    new Pose2d(8.272273, 1.914906, new Rotation2d())))
            .andThen(
                followTrajectory("olan"));
    }
}
