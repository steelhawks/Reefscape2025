package org.steelhawks.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.steelhawks.subsystems.vision.VisionConstants.APRIL_TAG_LAYOUT;

public class QuestNavImpl {

    private static final Matrix<N3, N1> STD_DEV = VecBuilder.fill(0.02, 0.02, 0.035);
    private static final double VISION_XY_DEVIATION_TOLERANCE = 0.3; // m
    private static final double VISION_THETA_DEVIATION_TOLERANCE = 0.087; // rad
    private static final Transform3d ROBOT_TO_QUEST =
        new Transform3d(0.0 , 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    private final Vision.VisionConsumer consumer;
    private final QuestNav nav;

    private boolean hasInitialPose = false;

    public QuestNavImpl(Vision.VisionConsumer consumer) {
        this.consumer = consumer;
        nav = new QuestNav();
    }

    public void periodic(List<Pose3d> visionPoses) {
        // required for library to work
        nav.commandPeriodic();

        List<PoseFrame> allPoseFrames = new LinkedList<>();
        List<Pose2d> allQuestPoses = new LinkedList<>();
        List<Pose2d> allQuestPosesAccepted = new LinkedList<>();
        List<Pose2d> allQuestPosesRejected = new LinkedList<>();

        PoseFrame[] frames = nav.getAllUnreadPoseFrames();
        allPoseFrames.addAll(Arrays.asList(frames));
        allQuestPoses.addAll(
            Arrays.stream(frames)
                .map(PoseFrame::questPose)
                .toList());
        if (frames.length > 0) {
            for (int i = 0; i < frames.length; i++) {
                PoseFrame frame = frames[i];
                Pose2d robotPose =
                    new Pose3d(frame.questPose())
                        .transformBy(ROBOT_TO_QUEST.inverse()).toPose2d();
                // filtering / compare questnav position to vision positioning
                final boolean rejectPose =
                    !hasInitialPose
                    || robotPose.getX() < 0.0
                    || robotPose.getX() > APRIL_TAG_LAYOUT.getFieldLength()
                    || robotPose.getY() < 0.0
                    || robotPose.getY() > APRIL_TAG_LAYOUT.getFieldWidth()
                    || outOfVisionTolerance(frame.questPose(),
                        visionPoses.stream().findAny().orElse(new Pose3d()).toPose2d())
                    || !nav.isTracking()
                    || !nav.isConnected();
                if (outOfVisionTolerance(frame.questPose(),
                    visionPoses.stream().findAny().orElse(new Pose3d()).toPose2d())
                ) {
                    boolean visionEstimateOk =
                        visionPoses.stream().findAny().orElse(new Pose3d()).getX() < 0.0
                        || visionPoses.stream().findAny().orElse(new Pose3d()).getX() > APRIL_TAG_LAYOUT.getFieldLength()
                        || visionPoses.stream().findAny().orElse(new Pose3d()).getY() < 0.0
                        || visionPoses.stream().findAny().orElse(new Pose3d()).getY() > APRIL_TAG_LAYOUT.getFieldWidth();
                    if (visionEstimateOk && !visionPoses.isEmpty()) {
                        setPose(visionPoses.get(0).toPose2d());
                    }
                }
                if (rejectPose) {
                    allQuestPosesRejected.add(robotPose);
                } else {
                    allQuestPosesAccepted.add(robotPose);
                }
                if (rejectPose) {
                    continue;
                }
                consumer.accept(robotPose, frame.dataTimestamp(), STD_DEV);

                Logger.recordOutput("QuestNav/AllPoses", allQuestPoses.toArray(new Pose2d[0]));
                Logger.recordOutput("QuestNav/RejectedPoses", allQuestPosesRejected.toArray(new Pose2d[0]));
                Logger.recordOutput("QuestNav/AcceptedPoses", allQuestPosesAccepted.toArray(new Pose2d[0]));
                Logger.recordOutput("QuestNav/Connected", nav.isConnected());
                Logger.recordOutput("QuestNav/Tracking", nav.isTracking());
                Logger.recordOutput("QuestNav/FrameCount", nav.getFrameCount().orElse(0));
                Logger.recordOutput("QuestNav/Battery", nav.getBatteryPercent().orElse(0));
                Logger.recordOutput("QuestNav/Latency", nav.getLatency());
            }
        }
    }

    private boolean outOfVisionTolerance(Pose2d questPose, Pose2d visionPose) {
        return !(Math.abs(questPose.getX() - visionPose.getX()) <= VISION_XY_DEVIATION_TOLERANCE)
            || !(Math.abs(questPose.getY() - visionPose.getY()) <= VISION_XY_DEVIATION_TOLERANCE)
            || !(Math.abs(questPose.getRotation().getRadians() - visionPose.getRotation().getRadians())
            <= VISION_THETA_DEVIATION_TOLERANCE);
    }

    public boolean isRunning() {
        return nav.isTracking() && nav.isConnected();
    }

    public void setPose(Pose2d pose) {
        hasInitialPose = true;
        nav.setPose(new Pose3d(pose).transformBy(ROBOT_TO_QUEST).toPose2d());
    }
}
