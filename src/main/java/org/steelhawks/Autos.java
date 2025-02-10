package org.steelhawks;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
                    new Pose2d(3.242988109588623, 4.184154510498047, new Rotation2d())))
            .andThen(DriveCommands.followPath(getPath("L1 Reef to Upper Algae")))
            .andThen(Commands.waitSeconds(1))
            .andThen(DriveCommands.followPath(getPath("Upper Algae to L2 Reef")))
            .andThen(Commands.waitSeconds(1))
            .andThen(DriveCommands.followPath(getPath("L1 Reef to Center Algae")))
            .andThen(Commands.waitSeconds(1))
            .andThen(DriveCommands.followPath(getPath("Center Algae to L2 Reef")))
            // .andThen(Commands.waitSeconds(1))
            // .andThen(DriveCommands.followPath(getPath("L2 Reef")))
            ;
    }

    public static Command getPathPlannerAuton() {
        return new PathPlannerAuto("Experimental Auto");
    }

    public static Command getFullPathPlannerAuto() {
        return new PathPlannerAuto("Full Auto");
    }

    public static Command getChoreoPathPlannerAuto() {
        return new PathPlannerAuto("Full Auto (Choreo)");
    }

    public static Command getShortChoreoPathPlannerAuto() {
        return new PathPlannerAuto("Short Auto (Choreo)");
    }


    // public static Command getHeitmanAuton() {
    //     return Commands.runOnce(
    //         () ->
    //             RobotContainer.s_Swerve.setPose(
    //                 new Pose2d(3.242988109588623, 4.184154510498047, new Rotation2d())))
    //         .andThen(
    //             DriveCommands.followPath(getPath("olan"))
    //         );    
    //     }

    public static PathPlannerPath getPath(String choreo) {
        try {
            return PathPlannerPath.fromChoreoTrajectory(choreo);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
