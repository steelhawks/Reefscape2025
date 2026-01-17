package org.steelhawks;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.Conversions;

import java.util.HashMap;


public final class ReefVisualizer {

    private static final double L4_HEIGHT = (183.0 - 6.0) / 100.0;
    private static final double L3_HEIGHT = (121.0 - 10.0) / 100.0;
    private static final double L2_HEIGHT = 0.81 - 0.1;

    private static final double L3_ALGAE = 1.31;
    private static final double L2_ALGAE = 0.91;

    private static final HashMap<String, Pose3d> coralPoses = new HashMap<>();
    private static final HashMap<String, Pose3d> algaePoses = new HashMap<>();
    private static int troughCount = 0;
    private static boolean previousFlipState = false;
    private static final Pose3d baselinePose =
        new Pose3d(
            Conversions.fromTranslation2dWithZ(ReefUtil.CoralBranch.L1.getBranchPoseProjectedToReefFace()
                .transformBy(new Transform2d(-0.05, 0.0, new Rotation2d())), L4_HEIGHT),
            new Rotation3d(0.0, Math.PI / 2.0, 0.0));

    static {
        for (AlgaePosition position : AlgaePosition.values()) {
            addAlgae(position);
        }
    }

    public enum CoralPosition {
        /* ------------- Left 1 ------------- */

        L4_L1(new Pose3d(
            Conversions.fromTranslation2dWithZ(
                ReefUtil.CoralBranch.L1.getBranchPoseProjectedToReefFace()
                    .transformBy(new Transform2d(-0.05, 0.0, new Rotation2d())),
                L4_HEIGHT),
            new Rotation3d(0.0, Math.PI / 2.0, 0.0))),

        L3_L1(new Pose3d(
            Conversions.fromTranslation2dWithZ(
                ReefUtil.CoralBranch.L1.getBranchPoseProjectedToReefFace()
                    .transformBy(new Transform2d(-0.13, 0.0, new Rotation2d())),
                L3_HEIGHT),
            new Rotation3d(0.0, Units.degreesToRadians(35.0), 0.0))),

        L2_L1(new Pose3d(
            Conversions.fromTranslation2dWithZ(
                ReefUtil.CoralBranch.L1.getBranchPoseProjectedToReefFace()
                    .transformBy(new Transform2d(-0.13, 0.0, new Rotation2d())),
                L2_HEIGHT),
            new Rotation3d(0.0, Units.degreesToRadians(35.0), 0.0))),

        /* ------------- Left 2 ------------- */

        L4_L2(L4_L1.getPosition().transformBy(
            new Transform3d(0.0, 2.0 * -FieldConstants.CENTER_OF_TROUGH_TO_BRANCH, 0.0, new Rotation3d()))),

        L3_L2(L3_L1.getPosition().transformBy(
            new Transform3d(0.0, 2.0 * -FieldConstants.CENTER_OF_TROUGH_TO_BRANCH, 0.0, new Rotation3d()))),

        L2_L2(L2_L1.getPosition().transformBy(
            new Transform3d(0.0, 2.0 * -FieldConstants.CENTER_OF_TROUGH_TO_BRANCH, 0.0, new Rotation3d()))),

        /* ------------- Top Left 1 ------------- */

        L4_TL1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L4_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    -Math.PI / 3.0)),
            L4_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, -Math.PI / 3.0)))),
        //        L4_TL2(
//            L4_L1.getPosition().rotateAround(new Translation3d(FieldConstants.REEF_CENTER), new Rotation3d(0.0, 0.0, Units.degreesToRadians(60.0)))
//            )

        L3_TL1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
            rotateAroundZ(
                L3_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
            -Math.PI / 3.0)),
            L3_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, -Math.PI / 3.0)))),

        L2_TL1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L2_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    -Math.PI / 3.0)),
            L2_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, -Math.PI / 3.0)))),

        /* ------------- Top Left 2 ------------- */

        L4_TL2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L4_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    -Math.PI / 3.0)),
            L4_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, -Math.PI / 3.0)))),

        L3_TL2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L3_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    -Math.PI / 3.0)),
            L3_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, -Math.PI / 3.0)))),

        L2_TL2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L2_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    -Math.PI / 3.0)),
            L2_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, -Math.PI / 3.0)))),

        /* ------------- Top Right 1 ------------- */

        L4_TR1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L4_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * -Math.PI / 3.0)),
            L4_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * -Math.PI / 3.0)))),

        L3_TR1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L3_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * -Math.PI / 3.0)),
            L3_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * -Math.PI / 3.0)))),

        L2_TR1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L2_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * -Math.PI / 3.0)),
            L2_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * -Math.PI / 3.0)))),

        /* ------------- Top Right 2 ------------- */

        L4_TR2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L4_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * -Math.PI / 3.0)),
            L4_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * -Math.PI / 3.0)))),

        L3_TR2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L3_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * -Math.PI / 3.0)),
            L3_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * -Math.PI / 3.0)))),

        L2_TR2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L2_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * -Math.PI / 3.0)),
            L2_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * -Math.PI / 3.0)))),

        /* ------------- Right 1 ------------- */

        L4_R1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L4_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI)),
            L4_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)))),

        L3_R1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L3_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI)),
            L3_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)))),

        L2_R1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L2_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI)),
            L2_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)))),

        /* ------------- Right 2 ------------- */

        L4_R2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L4_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI)),
            L4_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)))),

        L3_R2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L3_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI)),
            L3_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)))),

        L2_R2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L2_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI)),
            L2_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)))),

        /* ------------- Bottom Right 1 ------------- */

        L4_BR1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L4_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * Math.PI / 3.0)),
            L4_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * Math.PI / 3.0)))),

        L3_BR1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L3_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * Math.PI / 3.0)),
            L3_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * Math.PI / 3.0)))),

        L2_BR1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L2_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * Math.PI / 3.0)),
            L2_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * Math.PI / 3.0)))),

        /* ------------- Bottom Right 2 ------------- */

        L4_BR2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L4_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * Math.PI / 3.0)),
            L4_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * Math.PI / 3.0)))),

        L3_BR2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L3_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * Math.PI / 3.0)),
            L3_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * Math.PI / 3.0)))),

        L2_BR2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L2_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * Math.PI / 3.0)),
            L2_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * Math.PI / 3.0)))),

        /* ------------- Bottom Left 1 ------------- */

        L4_BL1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L4_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI / 3.0)),
            L4_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI / 3.0)))),
        //        L4_TL2(
//            L4_L1.getPosition().rotateAround(new Translation3d(FieldConstants.REEF_CENTER), new Rotation3d(0.0, 0.0, Units.degreesToRadians(60.0)))
//            )

        L3_BL1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L3_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI / 3.0)),
            L3_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI / 3.0)))),

        L2_BL1(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L2_L1.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI / 3.0)),
            L2_L1.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI / 3.0)))),

        /* ------------- Bottom Left 2 ------------- */

        L4_BL2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L4_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI / 3.0)),
            L4_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI / 3.0)))),

        L3_BL2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L3_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI / 3.0)),
            L3_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI / 3.0)))),

        L2_BL2(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L2_L2.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI / 3.0)),
            L2_L2.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI / 3.0))));

        private final Pose3d position;

        CoralPosition(Pose3d position) {
            this.position = position;
        }


        public Pose3d getPosition() {
            return AllianceFlip.apply(position);
        }
    }

    public enum AlgaePosition {
        L(new Pose3d(
            Conversions.fromTranslation2dWithZ(
                ReefUtil.Algae.L.getAprilTagPose()
                    .transformBy(new Transform2d(-0.15, 0.0, new Rotation2d())),
                L3_ALGAE),
            new Rotation3d(0.0, 0.0, 0.0))),

        TL(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    -Math.PI / 3.0
                ).plus(new Translation3d(0.0, 0.0, -L3_ALGAE + L2_ALGAE))),
            L.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, -Math.PI / 3.0)))),

        TR(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * -Math.PI / 3.0
                ).plus(new Translation3d(0.0, 0.0, 0.0))),
            L.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * -Math.PI / 3.0)))),

        R(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI
                ).plus(new Translation3d(0.0, 0.0, -L3_ALGAE + L2_ALGAE))),
            L.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)))),

        BR(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    2 * Math.PI / 3.0
                ).plus(new Translation3d(0.0, 0.0, 0.0))),
            L.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, 2 * Math.PI / 3.0)))),

        BL(new Pose3d(
            new Translation3d(FieldConstants.REEF_CENTER).plus(
                rotateAroundZ(
                    L.getPosition().getTranslation().minus(new Translation3d(FieldConstants.REEF_CENTER)),
                    Math.PI / 3.0
                ).plus(new Translation3d(0.0, 0.0, -L3_ALGAE + L2_ALGAE))),
            L.getPosition().getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI / 3.0))));

        private final Pose3d position;

        AlgaePosition(Pose3d position) {
            this.position = position;
        }

        public Pose3d getPosition() {
            return AllianceFlip.apply(position);
        }
    }

    public static Translation3d rotateAroundZ(Translation3d vec, double angleRadians) {
        double x = vec.getX();
        double y = vec.getY();
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);
        return new Translation3d(
            x * cos - y * sin,
            x * sin + y * cos,
            vec.getZ());
    }

    /**
     * Updates the visualizer to have a coral scored on the level.
     * @param state The level the coral should be scored on.
     * @param branch The branch the coral should be scored on.
     */
    public static void scoreCoral(ElevatorConstants.State state, String branch) {
        if ((state == ElevatorConstants.State.L4
            || state == ElevatorConstants.State.L3
            || state == ElevatorConstants.State.L2
            || state == ElevatorConstants.State.L1)
            && (branch.startsWith("L")
                || branch.startsWith("TL")
                || branch.startsWith("TR")
                || branch.startsWith("R")
                || branch.startsWith("BR")
                || branch.startsWith("BL"))
        ) {
            try {
                CoralPosition position = CoralPosition.valueOf(state.name() + "_" + branch);
                coralPoses.put(position.name(), position.getPosition());
            } catch (IllegalArgumentException e) {
                DriverStation.reportError("Invalid coral position: " + state.name() + "_" + branch, false);
            }
            return;
        }
        throw new IllegalArgumentException("The scoring level must be a reef level, (L1, L2, L3, L4) and a valid branch must be selected.");
    }

    /**
     * Updates the visualizer to have a coral scored on the level.
     * @param enumName The name as follows in the enum for the level and branch to score on.
     */
    public static void scoreCoral(String enumName) {
        if ((enumName.startsWith("L4")
            || enumName.startsWith("L3")
            || enumName.startsWith("L2")
            || enumName.startsWith("L1"))
            && enumName.matches(".*?(L|TL|TR|R|BR|BL)[12]?$")
        ) {
            try {
                CoralPosition position = CoralPosition.valueOf(enumName);
                coralPoses.put(position.name(), position.getPosition());
            } catch (IllegalArgumentException e) {
                DriverStation.reportError("Invalid coral position: " + enumName, false);
            }
            return;
        }
        throw new IllegalArgumentException("The scoring level must be a reef level, (L1, L2, L3, L4) and a valid branch must be selected.");
    }

    /**
     * Updates the visualizer to have coral scored in the trough.
     * @param troughCount The amount of trough scored
     */
    public static void scoreCoral(int troughCount) {
        troughCount += troughCount;
    }

    /**
     * Updates the visualizer to have a coral scored on the level.
     * @param position The position the coral should be scored on.
     */
    public static void scoreCoral(CoralPosition position) {
        coralPoses.put(position.name(), position.getPosition());
    }

    /**
     * Updates the visualizer to remove coral from a specific branch and level.
     * @param state The level the coral should be removed from.
     * @param branch The branch the coral should be removed from.
     */
    public void removeCoral(ElevatorConstants.State state, String branch) {
        if ((state == ElevatorConstants.State.L4
            || state == ElevatorConstants.State.L3
            || state == ElevatorConstants.State.L2
            || state == ElevatorConstants.State.L1)
            && (branch.startsWith("L")
            || branch.startsWith("TL")
            || branch.startsWith("TR")
            || branch.startsWith("R")
            || branch.startsWith("BR")
            || branch.startsWith("BL"))
        ) {
            try {
                coralPoses.remove(state.name() + "_" + branch);
            } catch (IllegalArgumentException e) {
                DriverStation.reportError("Invalid coral position: " + state.name() + "_" + branch, false);
            }
            return;
        }
        throw new IllegalArgumentException("The scoring level must be a reef level, (L1, L2, L3, L4) and a valid branch must be selected.");
    }

    /**
     * Updates the visualizer to have a coral scored on the level.
     * @param enumName The name as follows in the enum for the level and branch to score on.
     */
    public static void removeCoral(String enumName) {
        if ((enumName.startsWith("L4")
            || enumName.startsWith("L3")
            || enumName.startsWith("L2")
            || enumName.startsWith("L1"))
            && enumName.matches(".*?(L|TL|TR|R|BR|BL)[12]?$")
        ) {
            try {
                coralPoses.remove(enumName);
            } catch (IllegalArgumentException e) {
                DriverStation.reportError("Invalid coral position: " + enumName, false);
            }
            return;
        }
        throw new IllegalArgumentException("The scoring level must be a reef level, (L1, L2, L3, L4) and a valid branch must be selected.");
    }

    /**
     * Updates the visualizer to have coral removed from a coral position.
     * @param position The position the coral should be removed from.
     */
    public static void removeCoral(CoralPosition position) {
        coralPoses.remove(position.name());
    }

    /**
     * Updates the visualizer to have algae removed from the reef.
     * @param reefFace The reef face to remove the algae from.
     */
    public static void removeAlgae(String reefFace) {
        if (reefFace.equals("L")
            || reefFace.equals("TL")
            || reefFace.equals("TR")
            || reefFace.equals("R")
            || reefFace.equals("BR")
            || reefFace.equals("BL")
        ) {
            try {
                algaePoses.remove(reefFace);
            } catch (IllegalArgumentException e) {
                DriverStation.reportError("Invalid algae position: " + reefFace, false);
            }
            return;
        }
        throw new IllegalArgumentException("A valid branch must be selected");
    }

    /**
     * Updates the visualizer to have algae removed from the reef.
     * @param position The position to remove the algae from.
     */
    public static void removeAlgae(AlgaePosition position) {
        algaePoses.remove(position.name());
    }

    /**
     * Updates the visualizer to add algae back onto the reef.
     * @param reefFace The reef face to add the algae to.
     */
    public static void addAlgae(String reefFace) {
        if (reefFace.equals("L")
            || reefFace.equals("TL")
            || reefFace.equals("TR")
            || reefFace.equals("R")
            || reefFace.equals("BR")
            || reefFace.equals("BL")
        ) {
            try {
                algaePoses.put(reefFace, AlgaePosition.valueOf(reefFace).getPosition());
            } catch (IllegalArgumentException e) {
                DriverStation.reportError("Invalid algae position: " + reefFace, false);
            }
            return;
        }
        throw new IllegalArgumentException("A valid branch must be selected");
    }

    /**
     * Updates the visualizer to add algae back onto the reef.
     * @param position The position to add the algae to.
     */
    public static void addAlgae(AlgaePosition position) {
        algaePoses.put(position.name(), position.getPosition());
    }

    public static void updateVisualizer() {
        boolean currentFlipState = AllianceFlip.shouldFlip();
        if (currentFlipState != previousFlipState) {
            HashMap<String, Pose3d> newCoralPoses = new HashMap<>();
            for (var entry : coralPoses.entrySet()) {
                CoralPosition position = CoralPosition.valueOf(entry.getKey());
                newCoralPoses.put(position.name(), position.getPosition());
            }
            coralPoses.clear();
            coralPoses.putAll(newCoralPoses);

            HashMap<String, Pose3d> newAlgaePoses = new HashMap<>();
            for (var entry : algaePoses.entrySet()) {
                AlgaePosition position = AlgaePosition.valueOf(entry.getKey());
                newAlgaePoses.put(position.name(), position.getPosition());
            }
            algaePoses.clear();
            algaePoses.putAll(newAlgaePoses);

            previousFlipState = currentFlipState;
        }

        Logger.recordOutput("FieldSimulation/ReefCoral", getGamePieces("Coral"));
        Logger.recordOutput("FieldSimulation/ReefAlgae", getGamePieces("Algae"));
    }

    /**
     * Gets the game pieces as an array.
     * @param type The type of game piece to get, either "Coral" or "Algae"
     * @return Returns the array of game pieces, viewable in AdvantageScope
     */
    public static synchronized Pose3d[] getGamePieces(String type) {
        return type.equals("Coral") ? coralPoses.values().toArray(Pose3d[]::new) : algaePoses.values().toArray(Pose3d[]::new);
    }
}
