package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
import org.steelhawks.util.AllianceFlip;

import java.util.ArrayList;

public class Reefstate {

    private static final ArrayList<ReefSection> mReefSections = new ArrayList<>();

    static {
        for (int i = 0; i < 6; i++) {
            mReefSections.add(new ReefSection());
        }
    }

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

    public static int getClosestReefSection() {

        Pose2d[] allPoses = {
            Constants.FieldConstants.LEFT_SECTION,
            Constants.FieldConstants.TOP_LEFT_SECTION,
            Constants.FieldConstants.BOTTOM_LEFT_SECTION,
            Constants.FieldConstants.RIGHT_SECTION,
            Constants.FieldConstants.TOP_RIGHT_SECTION,
            Constants.FieldConstants.BOTTOM_RIGHT_SECTION
        };

        double minDist = Double.MAX_VALUE;
        int closestSection = 0;

        for (Pose2d pose : allPoses) {
            Pose2d validatedPose = AllianceFlip.validate(pose);
            double dist = RobotContainer.s_Swerve.getPose().getTranslation().getDistance(validatedPose.getTranslation());
            if (dist < minDist) {
                minDist = dist;

            }
        }

        return closestSection;
    }

    public static Pose2d getClosestReefSectionPose() {

        Pose2d[] allPoses = {
            Constants.FieldConstants.LEFT_SECTION,
            Constants.FieldConstants.TOP_LEFT_SECTION,
            Constants.FieldConstants.BOTTOM_LEFT_SECTION,
            Constants.FieldConstants.RIGHT_SECTION,
            Constants.FieldConstants.TOP_RIGHT_SECTION,
            Constants.FieldConstants.BOTTOM_RIGHT_SECTION
        };

        double minDist = Double.MAX_VALUE;
        Pose2d closestSectionPose2d = new Pose2d();

        for (Pose2d pose : allPoses) {
            Pose2d validatedPose = AllianceFlip.validate(pose);
            double dist = RobotContainer.s_Swerve.getPose().getTranslation().getDistance(validatedPose.getTranslation());
            if (dist < minDist) {
                minDist = dist;
                closestSectionPose2d = validatedPose;
            }
        }

        return closestSectionPose2d;
    }
}
