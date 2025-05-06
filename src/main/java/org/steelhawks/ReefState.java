package org.steelhawks;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.steelhawks.util.VirtualSubsystem;

public class ReefState extends VirtualSubsystem {
    private static final int LEVELS = 4;
    private static final int BRANCHES_PER_LEVEL = 6;

    private boolean[][] coralScored; // [level][branch]
    private boolean[] algaeRemoved;  // [level]
    private boolean coopertitionAchieved;

    @Override
    public void periodic() {
        updateFromNetworkTables();
        ReefVisualizer.updateVisualizer();
    }

    public ReefState() {
        coralScored = new boolean[LEVELS][BRANCHES_PER_LEVEL];
        algaeRemoved = new boolean[LEVELS];
        coopertitionAchieved = false;
    }

    public void updateFromNetworkTables() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("ReefData");

        for (int level = 0; level < LEVELS; level++) {
            for (int branch = 0; branch < BRANCHES_PER_LEVEL; branch++) {
                String key = "coral_" + level + "_" + branch;
                NetworkTableEntry entry = table.getEntry(key);
                coralScored[level][branch] = entry.getBoolean(false);
            }
            String algaeKey = "algae_" + level;
            NetworkTableEntry algaeEntry = table.getEntry(algaeKey);
            algaeRemoved[level] = algaeEntry.getBoolean(false);
        }

        NetworkTableEntry coopEntry = table.getEntry("coopertition");
        coopertitionAchieved = coopEntry.getBoolean(false);
    }

    public void updateVisualizer() {

    }
    
    public void score(int level, int branch) {
        coralScored[level][branch] = true;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("ReefData");
        String key = "coral_" + level + "_" + branch;
        table.getEntry(key).setBoolean(true);
    }

    public int[] getCoralCountsPerLevel() {
        int[] counts = new int[LEVELS];
        for (int level = 0; level < LEVELS; level++) {
            int count = 0;
            for (int branch = 0; branch < BRANCHES_PER_LEVEL; branch++) {
                if (coralScored[level][branch]) {
                    count++;
                }
            }
            counts[level] = count;
        }
        return counts;
    }

    public boolean isLevelAvailable(int level) {
        return algaeRemoved[level];
    }

    public int getBestLevelToScore() {
        int[] counts = getCoralCountsPerLevel();
        int minCount = Integer.MAX_VALUE;
        int bestLevel = -1;

        for (int level = 0; level < LEVELS; level++) {
            if (isLevelAvailable(level) && counts[level] < BRANCHES_PER_LEVEL) {
                if (counts[level] < minCount) {
                    minCount = counts[level];
                    bestLevel = level;
                }
            }
        }

        return bestLevel;
    }

    public boolean isCoralRPFeasible() {
        int[] counts = getCoralCountsPerLevel();
        int qualifyingLevels = 0;

        for (int count : counts) {
            if (count >= 5) {
                qualifyingLevels++;
            }
        }

        if (coopertitionAchieved) {
            return qualifyingLevels >= 3;
        } else {
            return qualifyingLevels == 4;
        }
    }

    public boolean isBranchAvailable(int level, int branch) {
        return isLevelAvailable(level) && !coralScored[level][branch];
    }

    public int[] getNextAvailableBranch(int level) {
        if (!isLevelAvailable(level)) {
            return null;
        }

        for (int branch = 0; branch < BRANCHES_PER_LEVEL; branch++) {
            if (!coralScored[level][branch]) {
                return new int[]{level, branch};
            }
        }

        return null;
    }

    public boolean isCoopertitionAchieved() {
        return coopertitionAchieved;
    }
}
