package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.steelhawks.Constants.FieldConstants;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.AllianceFlip;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class Reefstate {

    private static final List<FieldConstants.Position> reefPositions =
        Arrays.stream(FieldConstants.Position.values())
            .limit(6) // only take the first 6 positions for the reef
            .toList();

    private static final List<ReefSection> mReefSections =
        List.of(
            new ReefSection(), // Left Section
            new ReefSection(), // Top Left Section
            new ReefSection(), // Bottom Left Section
            new ReefSection(), // Right Section
            new ReefSection(), // Top Right Section
            new ReefSection() // Bottom Right Section
        );

    public static class ReefSection {

        public Branch mLeftBranch = new Branch();
        public Branch mRightBranch = new Branch();

        public int troughCount = 0;

        public boolean isSectionFull() {
            return mLeftBranch.isBranchFull() && mRightBranch.isBranchFull();
        }

        public class Branch {
            public boolean L2Occupied = false;
            public boolean L3Occupied = false;
            public boolean L4Occupied = false;

            public boolean isBranchFull() {
                return L2Occupied && L3Occupied && L4Occupied;
            }

            public void updateBranchState(boolean L2, boolean L3, boolean L4) {
                L2Occupied = L2;
                L3Occupied = L3;
                L4Occupied = L4;
            }

            public void l1() {
                troughCount++;
            }

            public void l2() {
                L2Occupied = true;
            }

            public void l3() {
                L3Occupied = true;
            }

            public void l4() {
                L4Occupied = true;
            }

            public void reset() {
                L2Occupied = false;
                L3Occupied = false;
                L4Occupied = false;
            }
        }
    }

    // Updates reef state when coral is placed
//    public static void placeCoral(int section, boolean leftBranch, boolean L2, boolean L3, boolean L4) {
//        ReefSection reefSection = mReefSections.get(section);
//        if (leftBranch) {
//            reefSection.mLeftBranch.updateBranchState(L2, L3, L4);
//        } else {
//            reefSection.mRightBranch.updateBranchState(L2, L3, L4);
//        }
//    }

    public static void placeCoral(int section, boolean leftBranch, ElevatorConstants.State level) {
        ReefSection reefSection = mReefSections.get(section);
        if (leftBranch) {
            switch (level) {
                case L1 -> reefSection.mLeftBranch.l1();
                case L2 -> reefSection.mLeftBranch.l2();
                case L3 -> reefSection.mLeftBranch.l3();
                case L4 -> reefSection.mLeftBranch.l4();
            }
        } else {
            switch (level) {
                case L1 -> reefSection.mRightBranch.l1();
                case L2 -> reefSection.mRightBranch.l2();
                case L3 -> reefSection.mRightBranch.l3();
                case L4 -> reefSection.mRightBranch.l4();
            }
        }
    }

    // Check if the entire reef is full
    public static boolean isReefFull() {
        for (ReefSection section : mReefSections) {
            if (!section.isSectionFull()) {
                return false;
            }
        }
        return true;
    }

    // Get a list of empty spots for coral placement
    public static ArrayList<String> getEmptySpots() {
        ArrayList<String> emptySpots = new ArrayList<>();
        for (int i = 0; i < mReefSections.size(); i++) {
            ReefSection section = mReefSections.get(i);
            if (!section.isSectionFull()) {
                if (!section.mLeftBranch.isBranchFull()) {
                    emptySpots.add("Section " + i + " - Left Branch");
                }
                if (!section.mRightBranch.isBranchFull()) {
                    emptySpots.add("Section " + i + " - Right Branch");
                }
            }
        }
        return emptySpots;
    }

    public static void updateReefState(int section, int troughCount) {
        ReefSection reefSection = mReefSections.get(section);
        reefSection.troughCount = troughCount;
    }

    public static Pose2d getClosestReef(Pose2d currentPose) {
        return reefPositions.stream()
            .min(Comparator.comparingDouble(reef ->
                AllianceFlip.apply(reef.getPose()).getTranslation().getDistance(currentPose.getTranslation())))
            .map(FieldConstants.Position::getPose)
            .orElse(null);
    }

    public static String getClosestReefName(Pose2d currentPose) {
        return reefPositions.stream()
            .min(Comparator.comparingDouble(reef ->
                AllianceFlip.apply(reef.getPose()).getTranslation().getDistance(currentPose.getTranslation())))
            .map(FieldConstants.Position::name)
            .orElse(null);
    }

    // fix
    public static ElevatorConstants.State getFreeLevel() {
        for (int i = 0; i < mReefSections.size(); i++) {
            ReefSection section = mReefSections.get(i);
            if (!section.isSectionFull()) {
                if (!section.mLeftBranch.isBranchFull()) {
                    return ElevatorConstants.State.L2;
                } else if (!section.mRightBranch.isBranchFull()) {
                    return ElevatorConstants.State.L3;
                }
            }
        }
        return ElevatorConstants.State.HOME;
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
