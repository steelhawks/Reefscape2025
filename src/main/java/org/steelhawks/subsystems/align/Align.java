package org.steelhawks.subsystems.align;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.Constants.AutonConstants;
import org.steelhawks.FieldConstants;
import org.steelhawks.ReefUtil;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.RobotContainer;
import org.steelhawks.commands.FusedSwerveDriveAlignment;
import org.steelhawks.commands.SwerveDriveAlignment;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.LED.LEDColor;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.VirtualSubsystem;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Align extends VirtualSubsystem {

    private static final double LEFT_TOLERANCE = 0.001;
    private static final double LEFT_KP = 0.8;
    private static final double LEFT_KI = 0;
    private static final double LEFT_KD = 0;

    private static final double RIGHT_TOLERANCE = 0.005;
    private static final double RIGHT_KP = 0.1;
    private static final double RIGHT_KI = 0;
    private static final double RIGHT_KD = 0;

    private static final double DIST_TOLERANCE = Units.inchesToMeters(0.005); // 0.00013 meters
    private static final double DIST_KP = 0.9;
    private static final double DIST_KI = 0;
    private static final double DIST_KD = 0;

    private static final double DIST_SPEED_MULTIPLIER = .2; // was .5
    private static final double LEFT_SPEED_MULTIPLIER = .3;
    private static final double RIGHT_SPEED_MULTIPLIER = .3;

    //    private static final double LEFT_SENSOR_ANGLE = 31; // degrees
    private static final double DISTANCE_BETWEEN_REEF_CORNER_AND_RIGHT_CORAL_BRANCH = 12.053354;
    private static final double DISTANCE_BETWEEN_REEF_CORNER_AND_LEFT_CORAL_BRANCH = 12.051569;
    private static final double DISTANCE_BETWEEN_CORAL_INTAKE_AND_ROBOT_RIGHT_SIDE = 6.101920;
    private static final double TARGET_DISTANCE = Units.inchesToMeters(3.0); // 0.051 meters
    private static final double LEFT_ALIGN_THRESHOLD = 0.39;
    private static final double RIGHT_ALIGN_THRESHOLD = 0.34500000000000003;

    private static final LoggedDashboardChooser<FieldConstants.Cage> cageChooser =
        new LoggedDashboardChooser<>("Cage Chooser");

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private final AlignIOInputsAutoLogged inputs = new AlignIOInputsAutoLogged();
    private final AlignIO io;

    private final PIDController mLeftController;
    private final PIDController mRightController;
    private final PIDController mDistanceController;
    private final Debouncer mDebouncer;

    private final Alert leftDisconnected;
    private final Alert rightDisconnected;

    public Align(AlignIO io) {
        this.io = io;

        cageChooser.addOption("Left", FieldConstants.Cage.LEFT);
        cageChooser.addOption("Right", FieldConstants.Cage.RIGHT);
        cageChooser.addOption("Center", FieldConstants.Cage.CENTER);

        mLeftController =
            new PIDController(LEFT_KP, LEFT_KI, LEFT_KD);
        mLeftController.setTolerance(LEFT_TOLERANCE);
        mRightController =
            new PIDController(RIGHT_KP, RIGHT_KI, RIGHT_KD);
        mRightController.setTolerance(RIGHT_TOLERANCE);
        mDistanceController =
            new PIDController(DIST_KP, DIST_KI, DIST_KD);
        mDistanceController.setTolerance(DIST_TOLERANCE);
        mDebouncer =
            new Debouncer(.5, DebounceType.kBoth);

        leftDisconnected = new Alert(
            "Left CANrange is disconnected", Alert.AlertType.kError);
        rightDisconnected = new Alert(
            "Right CANrange is disconnected", Alert.AlertType.kError);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Align", inputs);

        leftDisconnected.set(!inputs.leftConnected);
        rightDisconnected.set(!inputs.rightConnected);
    }

    public static PathPlannerPath directPath(Pose2d goal) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(s_Swerve.getPose(), goal);
        double speed =
            Math.hypot(
                s_Swerve.getChassisSpeeds().vxMetersPerSecond,
                s_Swerve.getChassisSpeeds().vyMetersPerSecond);
        PathPlannerPath path =
            new PathPlannerPath(
                waypoints,
                AutonConstants.CONSTRAINTS,
                new IdealStartingState(speed, s_Swerve.getRotation()),
                new GoalEndState(0, goal.getRotation()));
        path.preventFlipping = false;
        return path;
    }

    public static Command directPathFollow(Supplier<Pose2d> goal, boolean endsWhenAligned) {
        return DriveCommands.driveToPosition(goal.get())
            .andThen(new SwerveDriveAlignment(goal, endsWhenAligned));
    }

    public static Command directPathFollow(Supplier<Pose2d> goal) {
        return directPathFollow(goal, false);
    }

    public Command forwardUntil(Rotation2d angle) {
        return Commands.run(
            () -> {
                double output = mDistanceController.calculate(inputs.leftDistanceMeters, TARGET_DISTANCE);
                double alignOutput = s_Swerve.getAlign().calculate(s_Swerve.getRotation().getRadians(), angle.getRadians());
                Logger.recordOutput("Align/Feedback", output);
                Logger.recordOutput("Align/AngleFeedback", alignOutput);
                Logger.recordOutput("Align/AngleAtGoal", s_Swerve.getAlign().atGoal());

                if (!s_Swerve.getAlign().atGoal()) {
                    output = 0;
                }

                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        -output * s_Swerve.getMaxLinearSpeedMetersPerSec() * DIST_SPEED_MULTIPLIER,
                        0.0,
                        alignOutput);
                s_Swerve.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        AllianceFlip.shouldFlip()
                            ? s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                            : s_Swerve.getRotation()));
            }, s_Swerve)
        .until(() -> mDebouncer.calculate(mDistanceController.atSetpoint()))
        .finallyDo(() -> LED.getInstance().flashCommand(LEDColor.GREEN, .2, 2));
    }

    public Command alignLeft(Rotation2d angle) {
        return Commands.run(
            () -> {
                double alignOutput = s_Swerve.getAlign().calculate(s_Swerve.getRotation().getRadians(), angle.getRadians());
                double output = mLeftController.calculate(inputs.leftDistanceMeters, LEFT_ALIGN_THRESHOLD);
                Logger.recordOutput("Align/AngleFeedback", alignOutput);
                Logger.recordOutput("Align/LeftFeedback", output);
                s_Swerve.runVelocity(
                    new ChassisSpeeds(
                        0.0,
                        output * s_Swerve.getMaxLinearSpeedMetersPerSec() * LEFT_SPEED_MULTIPLIER,
                        alignOutput));
            }, s_Swerve)
        .until(() -> mDebouncer.calculate(mLeftController.atSetpoint()))
        .finallyDo(() -> LED.getInstance().flashCommand(LEDColor.GREEN, .2, 2));
    }

    public Command alignRight(Rotation2d angle) {
        return Commands.run(
            () -> {
                double alignOutput = s_Swerve.getAlign().calculate(s_Swerve.getRotation().getRadians(), angle.getRadians());
                double output = mRightController.calculate(inputs.rightDistanceMeters);
                Logger.recordOutput("Align/AngleFeedback", alignOutput);
                Logger.recordOutput("Align/RightFeedback", output);
                s_Swerve.runVelocity(
                    new ChassisSpeeds(
                        0.0,
                        output * s_Swerve.getMaxLinearSpeedMetersPerSec() * RIGHT_SPEED_MULTIPLIER,
                        0.0));
            }, s_Swerve)
        .until(() -> mDebouncer.calculate(mRightController.atSetpoint()))
        .finallyDo(() -> LED.getInstance().flashCommand(LEDColor.GREEN, .2, 2));
    }

    public Command alignToClosestReef(ElevatorConstants.State level) {
        return Commands.defer(
            () -> directPathFollow(() -> ReefUtil.getClosestCoralBranch().getScorePose(level)),
            Set.of(s_Swerve));
    }

    public Command alignToClosestReefWithFusedInput(ElevatorConstants.State level, DoubleSupplier joystickAxis) {
        return Commands.defer(
            () -> directPathFollow(() -> ReefUtil.getCoralBranchWithFusedDriverInput(joystickAxis).get().getScorePose(level)),
            Set.of(s_Swerve));
    }

    public Command alignToClosestAlgae() {
        return Commands.defer(
            () -> directPathFollow(() -> ReefUtil.getClosestAlgae().getRetrievePose()),
            Set.of(s_Swerve));
    }

    public Command alignToClosestCoralStation(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return Commands.defer(
            () -> DriveCommands.driveToPosition(FieldConstants.getClosestCoralStation().getIntakePoseViaPointToLine())
                .andThen(new FusedSwerveDriveAlignment(FieldConstants.getClosestCoralStation().getIntakePoseViaPointToLine(), xSupplier, ySupplier)),
            Set.of(s_Swerve));
    }

    public Command alignToClosestBargePoint(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return Commands.defer(
            () -> DriveCommands.driveToPosition(FieldConstants.Barge.SCORE.getClearancePose())
                .andThen(new FusedSwerveDriveAlignment(FieldConstants.Barge.SCORE.getClearancePose(), xSupplier, ySupplier)),
            Set.of(s_Swerve));
    }

    public Command alignToClosestCage() {
        return alignToCage(FieldConstants.getClosestCage());
    }

    public Command alignToSelectedCage() {
        return Commands.defer(
            () -> cageChooser.get() != null
                ? alignToCage(cageChooser.get())
                : alignToClosestCage(),
            Set.of(s_Swerve));
    }

    public Command alignToCage(FieldConstants.Cage cage) {
        return Commands.defer(
            () -> directPathFollow(cage::getClimbPose)
                .andThen(
                    Commands.run(
                            () -> s_Swerve.runVelocity(
                                new ChassisSpeeds(-0.3, 0.0, 0.0)))
                        .withTimeout(1.0)
                        .finallyDo(s_Swerve::stopWithX),
                    LED.getInstance().flashCommand(LEDColor.GREEN, 0.2, 3.0)),
            Set.of(s_Swerve));
    }
}
