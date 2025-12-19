package org.steelhawks.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.generated.TunerConstantsAlpha;
import org.steelhawks.generated.TunerConstantsHawkRider;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static edu.wpi.first.units.Units.*;
import static org.steelhawks.subsystems.vision.VisionConstants.APRIL_TAG_LAYOUT;

public class QuestNavImpl {

    private static final Matrix<N3, N1> STD_DEV = VecBuilder.fill(0.02, 0.02, 0.035);
    private static final double VISION_XY_DEVIATION_TOLERANCE = 0.3; // m
    private static final double VISION_THETA_DEVIATION_TOLERANCE = 0.087; // rad
    private static final Transform3d ROBOT_TO_QUEST =
        new Transform3d(0.0 , 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    private final Vision.VisionConsumer consumer;
    private final QuestNav nav;

    private static final LinearVelocity MAX_LINEAR_VELOCITY = switch (Constants.getRobot()) {
        case OMEGABOT, SIMBOT -> TunerConstants.kSpeedAt12Volts;
        case ALPHABOT -> TunerConstantsAlpha.kSpeedAt12Volts;
        case HAWKRIDER -> TunerConstantsHawkRider.kSpeedAt12Volts;
    };
    private static final AngularVelocity MAX_ANGULAR_VELOCITY =
        Constants.value(RadiansPerSecond.of(10.0), RadiansPerSecond.of(10.0));
    private static final LinearAcceleration MAX_LINEAR_ACCEL =
        Constants.value(MetersPerSecondPerSecond.of(8.0), MetersPerSecondPerSecond.of(8.0));
    private static final AngularAcceleration MAX_ANGULAR_ACCEL =
        Constants.value(RadiansPerSecondPerSecond.of(20.0), RadiansPerSecondPerSecond.of(20.0));
    private static final double MAX_LINEAR_JERK = Constants.value(40.0, 40.0);
    private static final double MAX_ANGULAR_JERK = Constants.value(40.0, 40.0);

    private Pose2d lastAcceptedPose = null;
    private double lastTimestamp = -1.0;
    private double lastLinearVelocity = 0.0;
    private double lastAngularVelocity = 0.0;
    private double lastLinearAccel = 0.0;
    private double lastAngularAccel = 0.0;
    private boolean hasInitialPose = false;

    public QuestNavImpl(Vision.VisionConsumer consumer) {
        this.consumer = consumer;
        nav = new QuestNav();
    }

    public void periodic() {
        // required for library to work
        nav.commandPeriodic();

        Logger.recordOutput("QuestNav/Connected", nav.isConnected());
        Logger.recordOutput("QuestNav/Tracking", nav.isTracking());
        Logger.recordOutput("QuestNav/FrameCount", nav.getFrameCount().orElse(0));
        Logger.recordOutput("QuestNav/Battery", nav.getBatteryPercent().orElse(0));
        Logger.recordOutput("QuestNav/Latency", nav.getLatency());

        List<PoseFrame> allPoseFrames = new LinkedList<>();
        List<Pose2d> allQuestPoses = new LinkedList<>();
        List<Pose2d> allQuestPosesAccepted = new LinkedList<>();
        List<Pose2d> allQuestPosesRejected = new LinkedList<>();

        PoseFrame[] frames = nav.getAllUnreadPoseFrames();
        allPoseFrames.addAll(Arrays.asList(frames));
        allQuestPoses.addAll(
            Arrays.stream(frames)
                .map(frame -> frame.questPose3d().toPose2d())
                .toList());
        if (frames.length > 0) {
            for (int i = 0; i < frames.length; i++) {
                PoseFrame frame = frames[i];
                Pose2d robotPose =
                    frame.questPose3d()
                        .transformBy(ROBOT_TO_QUEST.inverse()).toPose2d();
                // filtering / compare questnav position to vision positioning
                final boolean rejectPose =
                    Constants.loggedValue("HasInitialPose", !hasInitialPose)
                    || Constants.loggedValue("InXFieldMin",robotPose.getX() < 0.0)
                    || Constants.loggedValue("InXFieldMax", robotPose.getX() > APRIL_TAG_LAYOUT.getFieldLength())
                    || Constants.loggedValue("InYFieldMin", robotPose.getY() < 0.0)
                    || Constants.loggedValue("InYFieldMax", robotPose.getY() > APRIL_TAG_LAYOUT.getFieldWidth())
                    || Constants.loggedValue("PhysicallyRealisticMotion", !isPhysicallyFeasible(robotPose, frame.dataTimestamp()))
                    || Constants.loggedValue("QuestTracking", !nav.isTracking())
                    || Constants.loggedValue("QuestConnected", !nav.isConnected());
                Logger.recordOutput("QuestNav/UnfilteredPose", robotPose);
                if (rejectPose) {
                    allQuestPosesRejected.add(robotPose);
                } else {
                    allQuestPosesAccepted.add(robotPose);

                    // initialize on first accepted pose
                    if (lastAcceptedPose == null || lastTimestamp < 0) {
                        lastAcceptedPose = robotPose;
                        lastTimestamp = frame.dataTimestamp();
                        hasInitialPose = true;
                    } else {
                        double dt = frame.dataTimestamp() - lastTimestamp;

                        if (dt > 1e-6) {
                            double distance =
                                robotPose.getTranslation()
                                    .getDistance(lastAcceptedPose.getTranslation());
                            double linearVelo = distance / dt;

                            double angularVelo =
                                Math.abs(
                                    robotPose.getRotation()
                                        .minus(lastAcceptedPose.getRotation())
                                        .getRadians()
                                ) / dt;

                            double linearAccel =
                                (linearVelo - lastLinearVelocity) / dt;
                            double angularAccel =
                                (angularVelo - lastAngularVelocity) / dt;

                            double linearJerk =
                                (linearAccel - lastLinearAccel) / dt;
                            double angularJerk =
                                (angularAccel - lastAngularAccel) / dt;

                            // update derivative state
                            lastLinearVelocity = linearVelo;
                            lastAngularVelocity = angularVelo;
                            lastLinearAccel = linearAccel;
                            lastAngularAccel = angularAccel;

                            Logger.recordOutput("QuestNav/AcceptedLinearVel", linearVelo);
                            Logger.recordOutput("QuestNav/AcceptedAngularVel", angularVelo);
                            Logger.recordOutput("QuestNav/AcceptedLinearAccel", linearAccel);
                            Logger.recordOutput("QuestNav/AcceptedAngularAccel", angularAccel);
                            Logger.recordOutput("QuestNav/AcceptedLinearJerk", linearJerk);
                            Logger.recordOutput("QuestNav/AcceptedAngularJerk", angularJerk);
                        }

                        lastAcceptedPose = robotPose;
                        lastTimestamp = frame.dataTimestamp();
                    }
                }
                if (rejectPose) {
                    continue;
                }
                consumer.accept(robotPose, frame.dataTimestamp(), STD_DEV);

                Logger.recordOutput("QuestNav/AllPoses", allQuestPoses.toArray(new Pose2d[0]));
                Logger.recordOutput("QuestNav/RejectedPoses", allQuestPosesRejected.toArray(new Pose2d[0]));
                Logger.recordOutput("QuestNav/AcceptedPoses", allQuestPosesAccepted.toArray(new Pose2d[0]));
            }
        }
    }

    private boolean isPhysicallyFeasible(Pose2d newPose, double timestamp) {
        if (lastAcceptedPose == null || lastTimestamp < 0) {
            return true;
        }

        double dt = timestamp - lastTimestamp;
        if (dt <= 1e-6) return false;

        double distance =
            newPose.getTranslation().getDistance(lastAcceptedPose.getTranslation());
        double linearVelo = distance / dt;
        double angularDelta =
            Math.abs(
                newPose.getRotation()
                .minus(lastAcceptedPose.getRotation()).getRadians());
        double angularVelo = angularDelta / dt;
        double linearAccel =
            (linearVelo - lastLinearVelocity) / dt;
        double angularAccel = (angularVelo - lastAngularVelocity) / dt;
        double linearJerk =
            (linearAccel - lastLinearAccel) / dt;
        double angularJerk = (angularAccel - lastAngularAccel) / dt;

        Logger.recordOutput("QuestNav/LinearVel", linearVelo);
        Logger.recordOutput("QuestNav/AngularVel", angularVelo);
        Logger.recordOutput("QuestNav/LinearAccel", linearAccel);
        Logger.recordOutput("QuestNav/AngularAccel", angularAccel);
        Logger.recordOutput("QuestNav/LinearJerk", linearJerk);
        Logger.recordOutput("QuestNav/AngularJerk", angularJerk);

        return linearVelo <= MAX_LINEAR_VELOCITY.in(MetersPerSecond)
            && angularVelo <= MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)
            && Math.abs(linearAccel) <= MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond)
            && Math.abs(angularAccel) <= MAX_ANGULAR_ACCEL.in(RadiansPerSecondPerSecond)
//            && Math.abs(linearJerk) <= MAX_LINEAR_JERK
//            && Math.abs(angularJerk)  <= MAX_ANGULAR_JERK
        ;
    }

    private boolean outOfVisionTolerance(Pose2d questPose, Pose2d visionPose) {
        return Constants.loggedValue("VisionCheck/InXTolWithVision", !(Math.abs(questPose.getX() - visionPose.getX()) <= VISION_XY_DEVIATION_TOLERANCE))
            || Constants.loggedValue("VisionCheck/InYTolWithVision", !(Math.abs(questPose.getY() - visionPose.getY()) <= VISION_XY_DEVIATION_TOLERANCE))
            || Constants.loggedValue("VisionCheck/InThetaTolWithVision", !(Math.abs(questPose.getRotation().getRadians() - visionPose.getRotation().getRadians())
                <= VISION_THETA_DEVIATION_TOLERANCE));
    }

    public boolean isRunning() {
        return nav.isTracking() && nav.isConnected();
    }

    public void setPose(Pose2d pose) {
        hasInitialPose = true;
        nav.setPose(new Pose3d(pose).transformBy(ROBOT_TO_QUEST));
    }
}
