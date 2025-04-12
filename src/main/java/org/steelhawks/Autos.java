package org.steelhawks;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.commands.SuperStructure;
import org.steelhawks.commands.SwerveDriveAlignment;
import org.steelhawks.commands.autos.BottomG;
import org.steelhawks.commands.autos.RC2;
import org.steelhawks.commands.autos.TopG;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.autonbuilder.AutonBuilder;
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
    private static final AutonBuilder s_Builder = new AutonBuilder("Auton Builder");

    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Claw s_Claw = RobotContainer.s_Claw;

    public static final LoggedDashboardChooser<Command> sysIdChooser =
        new LoggedDashboardChooser<>("SysId Chooser");
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
        autoChooser.addDefaultOption("Nothing", Commands.none().withName("NOTHING_AUTO"));
        autoChooser.addOption("Use Auton Builder", Commands.none().withName("Use Auton Builder"));
        autoChooser.addOption("BC1", getBC1Auton());
        autoChooser.addOption("BC2", getBC2Auton());
        autoChooser.addOption("BC3", getBC3Auton());
        autoChooser.addOption("RC1", getRC1Auton());
        autoChooser.addOption("RC2", getRC2Auton());
        autoChooser.addOption("RC3", getRC3Auton());
        autoChooser.addOption("CENTER", getCenterAuton());
        autoChooser.addOption("TOPG", new TopG());
        autoChooser.addOption("BOTTOMG", new BottomG());
    }

    public static Misalignment getMisalignment() {
        String autoName = getAuto().getName();
        double radiansTolerance = Units.degreesToRadians(5);
        double xyTolerance = 0.6;

        double rotError = AllianceFlip.apply(new Rotation2d(StartEndPosition.valueOf(autoName).rotRadians)).getRadians()  - s_Swerve.getRotation().getRadians();
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
            SuperStructure.elevatorToPosition(state),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.waitUntil(s_Elevator.atThisGoal(state))),
            Commands.either(
                s_Claw.shootPulsatingCoral().withTimeout(0.6),
                s_Claw.shootCoral().withTimeout(0.6),
                () -> (state == ElevatorConstants.State.L1)),
                SuperStructure.elevatorToPosition(ElevatorConstants.State.HOME))
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
                            SuperStructure.elevatorToPosition(desiredScoreLevel)
                                .andThen(
                                    new SwerveDriveAlignment(() -> getScorePoseFromTrajectoryName(trajectory)).withTimeout(3.0),
                                    Commands.deadline(
                                        Commands.waitSeconds(1.2),
                                        Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
                                    Commands.either(
                                        s_Claw.shootCoralSlow().withTimeout(0.6),
                                        s_Claw.shootCoral().withTimeout(0.3),
                                        () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                                            desiredScoreLevel == ElevatorConstants.State.L4),
                                    SuperStructure.elevatorToPosition(ElevatorConstants.State.HOME)),
                            Commands.waitUntil(s_Claw.hasCoral()),
                            () -> atReef)));
        }

        return Commands.sequence(commands.toArray(new Command[commands.size()]));
    }


    private static Command createAuto(StartEndPosition pose, String... trajectories) {
        return Commands.runOnce(
            () -> s_Swerve.setPose(AllianceFlip.apply(pose.getPose())))
//            .andThen(s_Elevator.setDesiredState(desiredScoreLevel))
            .andThen(buildTrajectorySequence(trajectories));
    }

    public static Command getBC1AutonTest() {
        return Commands.runOnce(
            () -> s_Swerve.setPose(AllianceFlip.apply(StartEndPosition.BC1.getPose())))
            .andThen(
                followChoreoTrajectory("BC1 to TR2"),
                SuperStructure.elevatorToPosition(ElevatorConstants.State.L4),
                Commands.race(
                    Commands.waitSeconds(1),
                    Commands.waitUntil(s_Elevator.atGoal())),
                s_Claw.shootCoralSlow().withTimeout(1.0),
                SuperStructure.elevatorToPosition(ElevatorConstants.State.HOME))
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

    public static Command getBC1Auton() {
        return createAuto(StartEndPosition.BC1,
            "BC1 to TL2",
            "TL2 to Upper Source",
            "Upper Source to L1",
            "L1 to Upper Source",
            "Upper Source to L2"
            ).withName("BC1");
    }

    public static Command getBC2Auton() {
        return createAuto(StartEndPosition.BC2,
            "BC2 to TR1",
            "TR1 to Upper Source",
            "Upper Source to TL1",
            "TL1 to Upper Source",
            "Upper Source to TL2")
            .withName("BC2");
    }

    public static Command getBC3Auton() {
        return createAuto(StartEndPosition.BC3,
            "BC3 to R2",
            "R2 to Upper Source",
            "Upper Source to L2",
            "L2 to Upper Source",
            "Upper Source to L1",
            "L1 to Upper Source",
            "Upper Source to TL2")
            .withName("BC3");
    }

    public static Command getRC1Auton() {
//        return createAuto(StartEndPosition.RC1,
//            new String[]{
//                "RC1 to BR1",
//                "BR1 to Lower Source",
//                "Lower Source to BL1",
//                "BL1 to Lower Source",
//                "Lower Source to BL1",
//                "BL1 to Lower Source",
//                "Lower Source to BL2"
//            }).withName("RC1 Auto");
        return Commands.none();
    }

    public static Command getRC2Auton() {
//        return createAuto(StartEndPosition.RC2,
//            "RC2 to BR1", // was BR2
//            "BR2 to Lower Source",
//            "Lower Source to BL2",
//            "BL2 to Lower Source",
//            "Lower Source to BL1")
//            .withName("RC2");
        return new RC2();
    }

    public static Command getRC3Auton() {
//        return createAuto(StartEndPosition.RC3,
//            new String[]{
//                "RC3 to BR1",
//                "BR1 to Lower Source",
//                "Lower Source to BL1",
//                "BL1 to Lower Source",
//                "Lower Source to BL1",
//                "BL1 to Lower Source",
//                "Lower Source to BL2"
//            }).withName("RC3 Auto");
        return Commands.none();
    }

    public static Command getCenterAuton() {
        return createAuto(StartEndPosition.CENTER,
            "Center to R2"
            ).withName("CENTER");
    }

    public static Command getAuto() {
        if (autoChooser.get().getName().equals("Use Auton Builder")) {
            return s_Builder.getAutonCommand();
        } else if (Constants.TUNING_MODE) {
            return sysIdChooser.get();
        }
        return autoChooser.get();
    }
}
