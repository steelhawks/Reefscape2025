package org.steelhawks;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import org.dyn4j.geometry.Vector2;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.AprilTag;

public class FieldConstants {

    public static final double FIELD_LENGTH = VisionConstants.APRIL_TAG_LAYOUT.getFieldLength();
    public static final double FIELD_WIDTH = VisionConstants.APRIL_TAG_LAYOUT.getFieldWidth();

    /*
     * To properly use the auto flip feature, the poses MUST be for the blue alliance.
     * The auto flip feature will automatically flip the poses for the red alliance.
     */
    public static final Pose2d BLUE_STARTING_POSE =
        new Pose2d(new Translation2d(0, 0), new Rotation2d());

    public enum Position {
        LEFT_SECTION(new Pose2d(3.657600, 4.025900, new Rotation2d(Math.PI))),
        TOP_LEFT_SECTION(new Pose2d(4.073906, 4.745481, new Rotation2d(2 * Math.PI / 3))),
        BOTTOM_LEFT_SECTION(new Pose2d(4.073906, 3.306318, new Rotation2d(-2 * Math.PI / 3))),
        RIGHT_SECTION(new Pose2d(5.321046, 4.025900, new Rotation2d())),
        TOP_RIGHT_SECTION(new Pose2d(4.904739, 4.745481, new Rotation2d(Math.PI / 3))),
        BOTTOM_RIGHT_SECTION(new Pose2d(4.904739, 3.306318, new Rotation2d(-Math.PI / 3))),
        PROCESSOR(new Pose2d(5.987542, 0.407114, new Rotation2d(Math.PI / 2))),
        CORAL_STATION_BOTTOM(new Pose2d(1.007676, 1, new Rotation2d(0.94 + Math.PI))),
        CORAL_STATION_TOP(new Pose2d(1.007676, 6.96480, new Rotation2d(-0.94 + Math.PI)));

        private final Pose2d pose;

        Position(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    public static Translation2d REEF_CENTER = new Translation2d(Units.inchesToMeters(144.0 + (93.5 - 14.0 * 2) / 2), FIELD_WIDTH / 2);
    public static double CENTER_OF_REEF_TO_REEF_FACE = Units.inchesToMeters(32.75);
    public static double CENTER_OF_TROUGH_TO_BRANCH = Units.inchesToMeters(13.0 / 2.0);

    public static final AprilTag[] APRIL_TAGS = {
        new AprilTag(1,  new Pose3d(new Translation3d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80), Units.inchesToMeters(58.50)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(126)))),
        new AprilTag(2,  new Pose3d(new Translation3d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20), Units.inchesToMeters(58.50)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(234)))),
        new AprilTag(3,  new Pose3d(new Translation3d(Units.inchesToMeters(455.15), Units.inchesToMeters(317.15), Units.inchesToMeters(51.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(4,  new Pose3d(new Translation3d(Units.inchesToMeters(365.20), Units.inchesToMeters(241.64), Units.inchesToMeters(73.54)), new Rotation3d(Units.degreesToRadians(30), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(5,  new Pose3d(new Translation3d(Units.inchesToMeters(365.20), Units.inchesToMeters(75.39), Units.inchesToMeters(73.54)), new Rotation3d(Units.degreesToRadians(30), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(6,  new Pose3d(new Translation3d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(300)))),
        new AprilTag(7,  new Pose3d(new Translation3d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(8,  new Pose3d(new Translation3d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new AprilTag(9,  new Pose3d(new Translation3d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new AprilTag(10,  new Pose3d(new Translation3d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(11,  new Pose3d(new Translation3d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(240)))),
        new AprilTag(12,  new Pose3d(new Translation3d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), Units.inchesToMeters(58.50)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(54)))),
        new AprilTag(13,  new Pose3d(new Translation3d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), Units.inchesToMeters(58.50)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(306)))),
        new AprilTag(14,  new Pose3d(new Translation3d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64), Units.inchesToMeters(73.54)), new Rotation3d(Units.degreesToRadians(30), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(15,  new Pose3d(new Translation3d(Units.inchesToMeters(325.68), Units.inchesToMeters(75.39), Units.inchesToMeters(73.54)), new Rotation3d(Units.degreesToRadians(30), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(16,  new Pose3d(new Translation3d(Units.inchesToMeters(235.73), Units.inchesToMeters(-0.15), Units.inchesToMeters(51.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
        new AprilTag(17,  new Pose3d(new Translation3d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(240)))),
        new AprilTag(18,  new Pose3d(new Translation3d(Units.inchesToMeters(144.0), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(19,  new Pose3d(new Translation3d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new AprilTag(20,  new Pose3d(new Translation3d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new AprilTag(21,  new Pose3d(new Translation3d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(22,  new Pose3d(new Translation3d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(300)))),
    };

    public static AprilTag getAprilTag(int id) {
        return new AprilTag(id, VisionConstants.APRIL_TAG_LAYOUT.getTagPose(id).get());
    }

    public enum CoralStation {
        TOP(getAprilTag(13), getAprilTag(1), new Translation2d(0, FIELD_WIDTH / 2 + Units.inchesToMeters(109.13)), new Translation2d(Units.inchesToMeters(65.84), FIELD_WIDTH)),
        BOTTOM(getAprilTag(12), getAprilTag(2), new Translation2d(0, FIELD_WIDTH / 2 - Units.inchesToMeters(109.13)), new Translation2d(Units.inchesToMeters(65.84), 0));

        private final AprilTag blueTag;
        private final AprilTag redTag;
        private final Translation2d lineStart;
        private final Translation2d lineEnd;

        CoralStation(
            AprilTag blueTag, AprilTag redTag, Translation2d lineStart, Translation2d lineEnd) {
            this.blueTag = blueTag;
            this.redTag = redTag;
            this.lineStart = lineStart;
            this.lineEnd = lineEnd;
        }

        public Pose2d getAprilTagPose() {
            return AllianceFlip.shouldFlip() ? redTag.pose().toPose2d() : blueTag.pose().toPose2d();
        }

        public Pose2d getIntakePose(double yOffset) {
            return getAprilTagPose().transformBy(
                new Transform2d(
                    RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0,
                    yOffset,
                    new Rotation2d()));
        }

        public Pose2d getIntakePose() {
            return getIntakePose(0.0);
        }

        public double getDistanceToStation() {
            Vector2 A = new Vector2(getLineStart().getX(), getLineStart().getY());
            Vector2 B = new Vector2(getLineEnd().getX(), getLineEnd().getY());
            Vector2 C = new Vector2(
                RobotContainer.s_Swerve.getPose().getTranslation().getX(),
                RobotContainer.s_Swerve.getPose().getTranslation().getY());

            return Math.abs((C.x - A.x) * (-B.y + A.y) + (C.y - A.y) * (B.x - A.x))
                / Math.sqrt(Math.pow((-B.y + A.y), 2) + Math.pow((B.x - A.x), 2));
        }

        // https://www.youtube.com/watch?v=KHuI9bXZS74
        public Pose2d getIntakePoseViaPointToLine() {
            Translation2d startLine = getLineStart();
            Translation2d endLine = getLineEnd();
            Translation2d robotPoint = RobotContainer.s_Swerve.getPose().getTranslation();

            Vector2 lineVector = new Vector2(endLine.getX() - startLine.getX(), endLine.getY() - startLine.getY());
            Vector2 pointVector = new Vector2(robotPoint.getX() - startLine.getX(), robotPoint.getY() - startLine.getY());

            double lineLengthSquared = lineVector.dot(lineVector);
            double dotProduct = pointVector.dot(pointVector);

            double t = dotProduct / lineLengthSquared;
            double stationLength = startLine.getDistance(endLine);
            double percentToIgnoreFromEachSide = (RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0) / stationLength;

            t = Math.max(percentToIgnoreFromEachSide, Math.min(1 - percentToIgnoreFromEachSide, t));
            Translation2d closestPointOnCoralStation = new Translation2d(startLine.getX() + t * lineVector.x, startLine.getY() + t * lineVector.y);

            return new Pose2d(
                closestPointOnCoralStation,
                getAprilTagPose().getRotation())
            .transformBy(
                new Transform2d(
                    RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0,
                    0,
                    new Rotation2d()));
        }

        public Translation2d getLineStart() {
            return AllianceFlip.apply(lineStart);
        }

        public Translation2d getLineEnd() {
            return AllianceFlip.apply(lineEnd);
        }
    }

    public static CoralStation getClosestCoralStation() {
//        double distanceToTop =
//            RobotContainer.s_Swerve.getPose().getTranslation().getDistance(CoralStation.TOP.getAprilTagPose().getTranslation());
//        double distanceToBottom =
//            RobotContainer.s_Swerve.getPose().getTranslation().getDistance(CoralStation.BOTTOM.getAprilTagPose().getTranslation());

        return CoralStation.BOTTOM.getDistanceToStation() > CoralStation.TOP.getDistanceToStation() ? CoralStation.TOP : CoralStation.BOTTOM;
    }

    public static Pose2d getClosestCoralStationPose() {
        CoralStation closestStation = getClosestCoralStation();
        Pose2d robotPose = RobotContainer.s_Swerve.getPose();

        double[] offsets = {-0.5, -0.25, 0, 0.5, 0.25};

        Pose2d bestPose = null;
        double minDistance = Double.MAX_VALUE;

        for (double offset : offsets) {
            Pose2d intakePose = closestStation.getIntakePose(offset);
            double distance = robotPose.getTranslation().getDistance(intakePose.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                bestPose = intakePose;
            }
        }

        return bestPose;
    }
}
