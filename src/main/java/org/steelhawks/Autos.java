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

    public static AutoRoutine testAuton() {
        AutoRoutine testAuton = autoFactory.newRoutine("test");
        AutoTrajectory BC3_TO_R1 = testAuton.trajectory("BC3 to R1");

        testAuton.active().onTrue(
            Commands.sequence(
                BC3_TO_R1.resetOdometry(), 
                BC3_TO_R1.cmd()));

        return testAuton;
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

    public static Command getBC3ToR1Auto() {
        return Commands.runOnce(
            () -> s_Swerve.setPose(AllianceFlip.apply(StartEndPosition.BC3.getPose())))
            .andThen(
                followTrajectory("BC1 to TL1"),
                s_Elevator.setDesiredState(ElevatorConstants.State.L4),
                Commands.race(
                    Commands.waitSeconds(1),
                    Commands.waitUntil(s_Elevator.atGoal())),
                s_Intake.shootCoralSlow().withTimeout(1.0),
                s_Elevator.setDesiredState(ElevatorConstants.State.HOME),

                followTrajectory("TL1 to Upper Source"),

//                Commands.waitSeconds(1),

                followTrajectory("Upper Source to TL1")

//                s_Elevator.setDesiredState(ElevatorConstants.State.L4),
//                Commands.race(
//                    Commands.waitSeconds(1),
//                    Commands.waitUntil(s_Elevator.atGoal())),
//                s_Intake.shootCoralSlow().withTimeout(1.0),
//                s_Elevator.setDesiredState(ElevatorConstants.State.HOME),
//
//                followTrajectory("BL1 to Lower Source"),
//
//                Commands.waitSeconds(1),
//
//                followTrajectory("Lower Source to BL2"),
//                s_Elevator.setDesiredState(ElevatorConstants.State.L4),
//                Commands.race(
//                    Commands.waitSeconds(1),
//                    Commands.waitUntil(s_Elevator.atGoal())),
//                s_Intake.shootCoralSlow().withTimeout(1.0),
//                s_Elevator.setDesiredState(ElevatorConstants.State.HOME)
                );
    }

    public static Command test() {
        return Commands.runOnce(
                        () -> s_Swerve.setPose(AllianceFlip.apply(StartEndPosition.BC3.getPose())))
                .andThen(
                        followTrajectory("BC1 to TL1"),
                        s_Elevator.setDesiredState(ElevatorConstants.State.L4),
                        Commands.race(
                                Commands.waitSeconds(1),
                                Commands.waitUntil(s_Elevator.atGoal())),
                        s_Intake.shootCoralSlow()
                                .withTimeout(1.0)
                        ,
                        s_Elevator.setDesiredState(ElevatorConstants.State.HOME))


                .andThen(followTrajectory("TL1 to Upper Source"));
    }

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
