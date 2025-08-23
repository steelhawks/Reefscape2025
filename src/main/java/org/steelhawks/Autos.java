package org.steelhawks;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.commands.align.SwerveDriveAlignment;
import org.steelhawks.commands.autos.BC2_Pathless;
import org.steelhawks.commands.autos.RC2_Autopilot;
import org.steelhawks.commands.autos.RC2_Pathless;
import org.steelhawks.subsystems.algaeclaw.AlgaeClaw;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.autonbuilder.StartEndPosition;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import java.io.IOException;
import java.util.ArrayList;

@SuppressWarnings("unused")
public final class Autos {
    private static final ElevatorConstants.State desiredScoreLevel = ElevatorConstants.State.L4;

    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Claw s_Claw = RobotContainer.s_Claw;
    private static final AlgaeClaw s_AlgaeClaw = RobotContainer.s_AlgaeClaw;

    private static final LoggedDashboardChooser<Command> autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser");

    public enum Misalignment {
        NONE,
        ROTATION_CW,
        ROTATION_CCW,
        X_LEFT,
        X_RIGHT,
        Y_FORWARD,
        Y_BACKWARD,
        MULTIPLE
    }

    public static void init() {
        /* ------------- Autons ------------- */

        autoChooser.addDefaultOption("Nothing", Commands.none().withName("NOTHING_AUTO"));
        autoChooser.addOption("BC2", new BC2_Pathless(true));
        autoChooser.addOption("RC2 End BR1", new RC2_Autopilot(false));
        autoChooser.addOption("RC2 End L2", new RC2_Autopilot(true));
        autoChooser.addOption("Center R2", getCenterR2Auton());
        autoChooser.addOption("Center R1", getCenterR1Auton());

        if (Toggles.tuningMode.get()) {
            /* ------------- Swerve SysId ------------- */

            autoChooser.addOption("Swerve Drive (Quasistatic Forward)", s_Swerve.driveSysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Drive (Quasistatic Backward)", s_Swerve.driveSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Drive (Dynamic Forward)", s_Swerve.driveSysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Drive (Dynamic Backward)", s_Swerve.driveSysIdDynamic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Turn (Quasistatic Forward)", s_Swerve.turnSysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Turn (Quasistatic Backward)", s_Swerve.turnSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Turn (Dynamic Forward)", s_Swerve.turnSysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Turn (Dynamic Backward)", s_Swerve.turnSysIdDynamic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Angular (Quasistatic Forward)", s_Swerve.angularSysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Angular (Quasistatic Backward)", s_Swerve.angularSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Angular (Dynamic Forward)", s_Swerve.angularSysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Angular (Dynamic Backward)", s_Swerve.angularSysIdDynamic(SysIdRoutine.Direction.kReverse));

            /* ------------- Elevator SysId ------------- */

            autoChooser.addOption("Elevator (Quasistatic Forward)", s_Elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Elevator (Quasistatic Backward)", s_Elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Elevator (Dynamic Forward)", s_Elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Elevator (Dynamic Backward)", s_Elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

            /* ------------- AlgaeClaw SysId ------------- */

            autoChooser.addOption("AlgaeClaw (Quasistatic Forward)", s_AlgaeClaw.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("AlgaeClaw (Quasistatic Forward)", s_AlgaeClaw.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("AlgaeClaw (Dynamic Forward)", s_AlgaeClaw.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("AlgaeClaw (Dynamic Forward)", s_AlgaeClaw.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }
    }

    public static Misalignment getMisalignment() {
        if (Toggles.tuningMode.get()) {
            return Misalignment.NONE;
        }

        String autoName = getAuto().getName();
        double radiansTolerance = Units.degreesToRadians(5);
        double xyTolerance = 0.6;

        double rotError = AllianceFlip.apply(new Rotation2d(StartEndPosition.valueOf(autoName).rotRadians)).getRadians() - s_Swerve.getRotation().getRadians();
        double xError = AllianceFlip.applyX(StartEndPosition.valueOf(autoName).x) - s_Swerve.getPose().getX();
        double yError = AllianceFlip.applyY(StartEndPosition.valueOf(autoName).y) - s_Swerve.getPose().getY();

        boolean rotAligned = Math.abs(rotError) <= radiansTolerance;
        boolean xAligned = Math.abs(xError) <= xyTolerance;
        boolean yAligned = Math.abs(yError) <= xyTolerance;

        Logger.recordOutput(autoName + "/OmegaAligned", rotAligned);
        Logger.recordOutput(autoName + "/XAligned", xAligned);
        Logger.recordOutput(autoName + "/YAligned", yAligned);

        if (rotAligned && xAligned && yAligned) {
            return Misalignment.NONE;
        }

        if (!rotAligned && !xAligned && !yAligned) {
            return Misalignment.MULTIPLE;
        }

        if (!xAligned) {
            return (xError > 0) ? Misalignment.X_RIGHT : Misalignment.X_LEFT;
        }
        if (!yAligned) {
            return (yError > 0) ? Misalignment.Y_FORWARD : Misalignment.Y_BACKWARD;
        }

        return (rotError > 0) ? Misalignment.ROTATION_CCW : Misalignment.ROTATION_CW; // omega not being aligned is final scenario
    }

    public static Command followChoreoTrajectory(String choreo) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(choreo);
            return DriveCommands.followPath(path).withName("Following " + choreo);
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public static Command followTrajectory(String pathPlanner) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathPlanner);
            return DriveCommands.followPath(path).withName("Following " + pathPlanner);
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public static Command elevatorAndShoot(ElevatorConstants.State state) {
        return Commands.sequence(
            s_Elevator.setDesiredState(state),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.waitUntil(s_Elevator.atThisGoal(state))),
                s_Claw.shootCoral().withTimeout(0.6),
                s_Elevator.setDesiredState(ElevatorConstants.State.HOME))
        .withName("Elevator and Shoot in Auton");
    }

    private static boolean endsWithSource(String trajectory) {
        String[] words = trajectory.split(" ");
        return words[words.length - 1].equals("Source");
    }

    private static Pose2d getScorePoseFromTrajectoryName(String trajectory) {
        String[] words = trajectory.split(" ");
        return ReefUtil.CoralBranch.valueOf(words[words.length - 1]).getScorePose(desiredScoreLevel);
    }

    private static Command buildTrajectorySequence(String... trajectories) {
        ArrayList<Command> commands = new ArrayList<>();

        for (String trajectory : trajectories) {
            boolean atReef = !endsWithSource(trajectory);
            commands.add(
                followTrajectory(trajectory)
                    .andThen(
                        Commands.either(
                            s_Elevator.setDesiredState(desiredScoreLevel)
                                .andThen(
                                    new SwerveDriveAlignment(() -> getScorePoseFromTrajectoryName(trajectory)).withTimeout(3.0),
                                    Commands.deadline(
                                        Commands.waitSeconds(1.2),
                                        Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
                                        s_Claw.shootCoral().withTimeout(0.3),
                                    s_Elevator.setDesiredState(ElevatorConstants.State.HOME)),
                            Commands.waitUntil(s_Claw::hasCoral),
                            () -> atReef)));
        }

        return Commands.sequence(commands.toArray(new Command[commands.size()]));
    }


    private static Command createAuto(StartEndPosition pose, String... trajectories) {
        return Commands.runOnce(
            () -> s_Swerve.setPose(AllianceFlip.apply(pose.getPose())))
            .andThen(buildTrajectorySequence(trajectories));
    }

    public static Command getBC1AutonTest() {
        return Commands.runOnce(
            () -> s_Swerve.setPose(AllianceFlip.apply(StartEndPosition.BC1.getPose())))
            .andThen(
                followChoreoTrajectory("BC1 to TR2"),
                s_Elevator.setDesiredState(ElevatorConstants.State.L4),
                Commands.race(
                    Commands.waitSeconds(1),
                    Commands.waitUntil(s_Elevator.atGoal())),
                s_Claw.shootCoralEnd(),
                s_Elevator.setDesiredState(ElevatorConstants.State.HOME))
            .andThen(followChoreoTrajectory("TR2 to Upper Source"));
    }

    public static Command getStraightTestPath() {
        return new PathPlannerAuto("straight auto");
    }

    public static Command getTurnTestPath() {
        return new PathPlannerAuto("turn auto");
    }

    public static Command getCurvedTestPath() {
        return new PathPlannerAuto("curved auto");
    }

    public static Command getCenterR2Auton() {
        return createAuto(StartEndPosition.CENTER,
            "Center to R2")
            .withName("CENTER");
    }

    public static Command getCenterR1Auton() {
        return createAuto(StartEndPosition.CENTER,
            "Center to R1")
            .withName("CENTER");
    }

    public static Command getAuto() {
        return autoChooser.get();
    }
}
