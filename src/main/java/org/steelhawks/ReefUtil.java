package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants.Deadbands;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.AprilTag;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ReefUtil {

    private static final List<FieldConstants.Position> reefPositions =
        Arrays.stream(FieldConstants.Position.values())
            .limit(6)
            .toList();

    public enum CoralBranch {
        TR1(FieldConstants.getAprilTag(20), FieldConstants.getAprilTag(11)),
        TR2(FieldConstants.getAprilTag(20), FieldConstants.getAprilTag(11)),
        R1(FieldConstants.getAprilTag(21), FieldConstants.getAprilTag(10)),
        R2(FieldConstants.getAprilTag(21), FieldConstants.getAprilTag(10)),
        BR1(FieldConstants.getAprilTag(22), FieldConstants.getAprilTag(9)),
        BR2(FieldConstants.getAprilTag(22), FieldConstants.getAprilTag(9)),
        BL1(FieldConstants.getAprilTag(17), FieldConstants.getAprilTag(8)),
        BL2(FieldConstants.getAprilTag(17), FieldConstants.getAprilTag(8)),
        L1(FieldConstants.getAprilTag(18), FieldConstants.getAprilTag(7)),
        L2(FieldConstants.getAprilTag(18), FieldConstants.getAprilTag(7)),
        TL1(FieldConstants.getAprilTag(19), FieldConstants.getAprilTag(6)),
        TL2(FieldConstants.getAprilTag(19), FieldConstants.getAprilTag(6));

        private final AprilTag blueTag;
        private final AprilTag redTag;

        CoralBranch(AprilTag blueTag, AprilTag redTag) {
            this.blueTag = blueTag;
            this.redTag = redTag;
        }

        public boolean isLeftBranch() {
            return switch (this) {
                case TR1, R1, BR1, BL1, L1, TL1 -> true;
                default -> false;
            };
        }

        public Pose2d getAprilTagPose() {
            return AllianceFlip.shouldFlip() ? redTag.pose().toPose2d() : blueTag.pose().toPose2d();
        }

        public Pose2d getBranchPoseProjectedToReefFace() {
            return getAprilTagPose().transformBy(
                new Transform2d(
                    0.0,
                    FieldConstants.CENTER_OF_TROUGH_TO_BRANCH * (isLeftBranch() ? -1 : 1),
                    new Rotation2d()));
        }

        public Pose2d getScorePose(ElevatorConstants.State level) {
            double distFromReef = Units.inchesToMeters(
                switch (level) {
                    case L1 -> 0.5;
                    case L2 -> 0.0; // find the distance from the reef to the branch
                    case L3 -> 0.0;
                    case L4 -> 0.0;
                    default -> throw new IllegalArgumentException("Invalid level: " + level);
            });

            return level != ElevatorConstants.State.L1
                ? getAprilTagPose().transformBy(
                new Transform2d(
                    RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0 + distFromReef,
                    (FieldConstants.CENTER_OF_TROUGH_TO_BRANCH * (isLeftBranch() ? -1.0 : 1.0)) + RobotConstants.CLAW_Y_OFFSET,
                    new Rotation2d(Math.PI)))
                : getAprilTagPose().transformBy(
                    new Transform2d(
                        RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0 + distFromReef,
                        FieldConstants.ROBOT_PERPENDICULAR_TO_NEXT_REEF * (isLeftBranch() ? 1.0 : -1.0)
                            + RobotConstants.CLAW_Y_OFFSET * (isLeftBranch() ? 1.0 : -1.0),
                        new Rotation2d(Math.PI)));
        }
    }

    public static CoralBranch getClosestCoralBranch() {
        CoralBranch nearestBranch = CoralBranch.TR1;
        double closestDistance = Double.MAX_VALUE;

        for (CoralBranch branch : CoralBranch.values()) {
            double distance = RobotContainer.s_Swerve.getPose()
                .minus(branch.getBranchPoseProjectedToReefFace())
                .getTranslation()
                .getNorm();
            if (distance < closestDistance) {
                closestDistance = distance;
                nearestBranch = branch;
            }
        }

        return nearestBranch;
    }

    public static Supplier<CoralBranch> getCoralBranchWithFusedDriverInput(DoubleSupplier joystickAxis) {
        return () -> {
            if (joystickAxis.getAsDouble() > Deadbands.BRANCH_OVERRIDE_DEADBAND) {
                return switch (ReefUtil.getClosestCoralBranch()) {
                    case TR1, TR2 -> CoralBranch.TR2;
                    case R1, R2 -> CoralBranch.R2;
                    case BR1, BR2 -> CoralBranch.BR2;
                    case BL1, BL2 -> CoralBranch.BL2;
                    case L1, L2 -> CoralBranch.L2;
                    case TL1, TL2 -> CoralBranch.TL2;
                };
            } else if (joystickAxis.getAsDouble() < -Deadbands.BRANCH_OVERRIDE_DEADBAND) {
                return switch (ReefUtil.getClosestCoralBranch()) {
                    case TR1, TR2 -> CoralBranch.TR1;
                    case R1, R2 -> CoralBranch.R1;
                    case BR1, BR2 -> CoralBranch.BR1;
                    case BL1, BL2 -> CoralBranch.BL1;
                    case L1, L2 -> CoralBranch.L1;
                    case TL1, TL2 -> CoralBranch.TL1;
                };
            } else {
                return ReefUtil.getClosestCoralBranch();
            }
        };
    }

    public static String getClosestCoralBranchName() {
        return getClosestCoralBranch().name();
    }

    public enum Algae {
        TR(FieldConstants.getAprilTag(20), FieldConstants.getAprilTag(11)),
        R(FieldConstants.getAprilTag(21), FieldConstants.getAprilTag(10)),
        BR(FieldConstants.getAprilTag(22), FieldConstants.getAprilTag(9)),
        BL(FieldConstants.getAprilTag(17), FieldConstants.getAprilTag(8)),
        L(FieldConstants.getAprilTag(18), FieldConstants.getAprilTag(7)),
        TL(FieldConstants.getAprilTag(19), FieldConstants.getAprilTag(6));

        private final AprilTag blueTag;
        private final AprilTag redTag;

        Algae(AprilTag blueTag, AprilTag redTag) {
            this.blueTag = blueTag;
            this.redTag = redTag;
        }

        public boolean isOnL3() {
            return switch (this) {
                case TR, BR, L -> true;
                default -> false;
            };
        }

        public Pose2d getAprilTagPose() {
            return AllianceFlip.shouldFlip() ? redTag.pose().toPose2d() : blueTag.pose().toPose2d();
        }

        public Pose2d getRetrievePose() {
            return getAprilTagPose().transformBy(
                new Transform2d(
                    RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0,
                    RobotConstants.ALGAE_CLAW_Y_OFFSET,
                    new Rotation2d(Math.PI / 2.0)));
        }

        public Pose2d getClearancePose() {
            return getRetrievePose().transformBy(
                new Transform2d(
                    0.0,
                    -Units.inchesToMeters(13),
                    new Rotation2d()));
        }
    }

    public static Algae getClosestAlgae() {
        Algae nearestAlgae = Algae.TR;
        double closestDistance = Double.MAX_VALUE;

        for (Algae algae : Algae.values()) {
            double distance = RobotContainer.s_Swerve.getPose().minus(algae.getRetrievePose()).getTranslation().getNorm();
            if (distance < closestDistance) {
                closestDistance = distance;
                nearestAlgae = algae;
            }
        }

        return nearestAlgae;
    }

    public static String getClosestAlgaeName() {
        return getClosestAlgae().name();
    }

    public static Rotation2d getRotationFromTagId(int tagId) {
        if (AllianceFlip.shouldFlip()) { // red alliance
            return switch (tagId) {
                case 17 -> FieldConstants.Position.BOTTOM_LEFT_SECTION.getPose().getRotation();
                case 18 -> FieldConstants.Position.LEFT_SECTION.getPose().getRotation();
                case 19 -> FieldConstants.Position.TOP_LEFT_SECTION.getPose().getRotation();
                case 20 -> FieldConstants.Position.TOP_RIGHT_SECTION.getPose().getRotation();
                case 21 -> FieldConstants.Position.RIGHT_SECTION.getPose().getRotation();
                case 22 -> FieldConstants.Position.BOTTOM_RIGHT_SECTION.getPose().getRotation();
                default -> null;
            };
        } else { // blue allance
            return switch (tagId) {
                case 6 -> FieldConstants.Position.BOTTOM_RIGHT_SECTION.getPose().getRotation();
                case 7 -> FieldConstants.Position.RIGHT_SECTION.getPose().getRotation();
                case 8 -> FieldConstants.Position.TOP_RIGHT_SECTION.getPose().getRotation();
                case 9 -> FieldConstants.Position.TOP_LEFT_SECTION.getPose().getRotation();
                case 10 -> FieldConstants.Position.LEFT_SECTION.getPose().getRotation();
                case 11 -> FieldConstants.Position.BOTTOM_LEFT_SECTION.getPose().getRotation();
                default -> null;
            };
        }
    }
}
