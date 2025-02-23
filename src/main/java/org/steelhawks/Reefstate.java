package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.steelhawks.Constants.FieldConstants;
import org.steelhawks.util.AllianceFlip;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class Reefstate {

    private static final List<FieldConstants.Position> reefPositions =
        Arrays.stream(FieldConstants.Position.values())
            .limit(6) // only take the first 6 positions for the reef
//            .map(FieldConstants.Position::getPose)
            .toList();

    private static final List<ReefSection> mReefSections =
        List.of(
            new ReefSection(),
            new ReefSection(),
            new ReefSection(),
            new ReefSection(),
            new ReefSection(),
            new ReefSection()
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
        }
    }

    // Updates reef state when coral is placed
    public static void placeCoral(int section, boolean leftBranch, boolean L2, boolean L3, boolean L4) {
        ReefSection reefSection = mReefSections.get(section);
        if (leftBranch) {
            reefSection.mLeftBranch.updateBranchState(L2, L3, L4);
        } else {
            reefSection.mRightBranch.updateBranchState(L2, L3, L4);
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

    @AutoLogOutput(key = "Pose/ClosestReef")
    public static Pose2d getClosestReef(Pose2d currentPose) {
        return reefPositions.stream()
            .min(Comparator.comparingDouble(reef ->
                AllianceFlip.apply(reef.getPose()).getTranslation().getDistance(currentPose.getTranslation())))
            .map(FieldConstants.Position::getPose)
            .orElse(null);
    }
}
