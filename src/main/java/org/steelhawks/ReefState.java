package org.steelhawks;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.steelhawks.ReefUtil.CoralBranch;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.VirtualSubsystem;

import java.util.*;

public class ReefState extends VirtualSubsystem {
    private static final String[] REEF_NAMES = {
        "leftOne", "leftTwo",
        "topLeftOne", "topLeftTwo",
        "topRightOne", "topRightTwo",
        "rightOne", "rightTwo",
        "bottomRightOne", "bottomRightTwo",
        "bottomLeftOne", "bottomLeftTwo"
    };

    private static final String[] levels = {
        "L4",
        "L3",
        "L2"
    };

    private static final Map<String,String> BRANCH_CODE = Map.ofEntries(
        Map.entry("leftOne", "L1"),
        Map.entry("leftTwo", "L2"),
        Map.entry("topLeftOne", "TL1"),
        Map.entry("topLeftTwo", "TL2"),
        Map.entry("topRightOne", "TR1"),
        Map.entry("topRightTwo", "TR2"),
        Map.entry("rightOne", "R1"),
        Map.entry("rightTwo", "R2"),
        Map.entry("bottomRightOne", "BR1"),
        Map.entry("bottomRightTwo", "BR2"),
        Map.entry("bottomLeftOne", "BL1"),
        Map.entry("bottomLeftTwo", "BL2"));

    private String toBranchCode(String reefName) {
        return BRANCH_CODE.getOrDefault(reefName, reefName);
    }


    private static final String troughKey = "TroughCount";
    private static final String coopKey = "coop";

    // for levels 2–4 each reef has 3 branches; level1 uses troughCount
    private Map<String, boolean[]> coralMap;
    private int troughCount;
    private boolean coop;

    public ReefState() {
        coralMap = new HashMap<>();
        for (String name : REEF_NAMES) {
            coralMap.put(name, new boolean[3]);
        }
        troughCount = 0;
        coop = false;
    }

    @Override
    public void periodic() {
        updateFromNetworkTables();
        syncVisualizer(); // ← push our boolean‑map into the visualizer
        ReefVisualizer.updateVisualizer();
    }

    /**
     * Push our internal coralMap & troughCount into the 3D visualizer.
     */
    private void syncVisualizer() {
        // 1) clear all previously scored coral
        for (String name : coralMap.keySet()) {
            // remove any enum entries like "L4_L1" etc.
            for (String level : levels) {
                ReefVisualizer.removeCoral(level + "_" + toBranchCode(name));
            }

        }
        // if you want to visualize troughCount (level1) as L1 branches:
//        for (int i = 0; i < troughCount; i++) {
//            // e.g. name them "L1_branch0", "L1_branch1", …
//            ReefVisualizer.scoreCoral(i);
//        }

        // 2) re‑score every coral toggle for levels2–4
        for (var entry : coralMap.entrySet()) {
            String reefName = entry.getKey();
            boolean[] levels = entry.getValue();
            for (int lvlIdx = 0; lvlIdx < levels.length; lvlIdx++) {
                if (levels[lvlIdx]) {
                    // map lvlIdx → dashboard level: 0→4, 1→3, 2→2
                    int dashLevel = (lvlIdx == 0 ? 4 : lvlIdx == 1 ? 3 : 2);
                    // reefName is e.g. "leftOne", prepend "L4_", "L3_" or "L2_"
                    String enumName = "L" + dashLevel + "_" + toBranchCode(reefName);
                    ReefVisualizer.scoreCoral(enumName);
                }
            }
        }
    }

    public void updateFromNetworkTables() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("ReefData");

        // read troughCount
        troughCount = (int) table.getEntry(troughKey).getInteger(0);

        // read coop flag
        coop = table.getEntry(coopKey).getBoolean(false);

        // read every coral array entry
        for (String name : REEF_NAMES) {
            boolean[] arr = coralMap.get(name);
            for (int idx = 0; idx < arr.length; idx++) {
                String key = name + "_" + idx;
                arr[idx] = table.getEntry(key).getBoolean(false);
            }
        }
    }

    /**
     * Called from robot code when you score a coral.
     */
    public void scoreCoral(String reefName, int levelIndex) {
        if (!coralMap.containsKey(reefName) || levelIndex < 0 || levelIndex >= 3) return;
        coralMap.get(reefName)[levelIndex] = true;
        // push to NetworkTables
        NetworkTable table = NetworkTableInstance.getDefault().getTable("ReefData");
        table.getEntry(reefName + "_" + levelIndex).setBoolean(true);
    }

    /**
     * Called when you change troughCount (level1).
     */
    public void setTroughCount(int count) {
        troughCount = count;
        NetworkTableInstance.getDefault()
            .getTable("ReefData")
            .getEntry("troughCount")
            .setNumber(count);
    }

    /**
     * Called when coop toggles.
     */
    public void setCoop(boolean c) {
        coop = c;
        NetworkTableInstance.getDefault()
            .getTable("ReefData")
            .getEntry(coopKey)
            .setBoolean(c);
    }

    /**
     * Returns total coral‑toggles at dashboard level (4→index0, 3→1, 2→2, 1→troughCount).
     */
    public int getCountForLevel(int level) {
        switch (level) {
            case 4:
            case 3:
            case 2:
                int idx = (level == 4 ? 0 : level == 3 ? 1 : 2);
                int sum = 0;
                for (boolean[] arr : coralMap.values()) {
                    if (arr[idx]) sum++;
                }
                return sum;
            case 1:
                return troughCount;
            default:
                return 0;
        }
    }

    /**
     * “minimum” is 7 (or 5 in coop).
     */
    public boolean achievedLevelMinimum(int level) {
        int needed = (coop ? 5 : 7);
        return getCountForLevel(level) >= needed;
    }

    /**
     * all levels 2–4 ≥7 (or 5 if coop).
     */
    public boolean achievedCoopertition() {
        for (int lvl = 2; lvl <= 4; lvl++) {
            if (!achievedLevelMinimum(lvl)) return false;
        }
        return true;
    }

    /**
     * Coral RP: either 4 levels ≥7 (no‑coop) or 3 levels ≥7 + coop.
     */
    public boolean isCoralRPFeasible() {
        int qualified = 0;
        for (int lvl = 2; lvl <= 4; lvl++) {
            if (getCountForLevel(lvl) >= 7) qualified++;
        }
        return coop ? (qualified >= 3) : (qualified == 3);
    }

    /**
     * pick next branch on “best” level.
     */
    public String[] getNextToggle() {
        // find level with lowest count that’s available
        int bestLevel = -1, min = Integer.MAX_VALUE;
        for (int lvl = 2; lvl <= 4; lvl++) {
            int cnt = getCountForLevel(lvl);
            if (cnt < min) {
                min = cnt;
                bestLevel = lvl;
            }
        }
        if (bestLevel < 2) return null;
        int idx = bestLevel == 4 ? 0 : bestLevel == 3 ? 1 : 2;
        // find any reefName where coralMap.get(name)[idx]==false
        for (String name : REEF_NAMES) {
            if (!coralMap.get(name)[idx]) {
                return new String[]{name, String.valueOf(idx)};
            }
        }
        return null;
    }
    
    public ScoreGoal getNextBestBranch() {
        ArrayList<CoralBranch> sortedByDistance = new ArrayList<>(Arrays.asList(CoralBranch.values()));
        sortedByDistance.sort((b1, b2) -> {
            double d1 = RobotContainer.s_Swerve.getPose()
                .minus(b1.getBranchPoseProjectedToReefFace())
                .getTranslation()
                .getNorm();
            double d2 = RobotContainer.s_Swerve.getPose()
                .minus(b2.getBranchPoseProjectedToReefFace())
                .getTranslation()
                .getNorm();
            return Double.compare(d1, d2);
        });

        // Find first unscored branch
//        for (CoralBranch branch : sortedByDistance) {
//            get
//            branch.name()
//        }

        return null;
    }

    // getters for dashboard
    public Map<String, boolean[]> getAllCoralMaps() {
        return coralMap;
    }

    public int getTroughCount() {
        return troughCount;
    }

    public boolean isCoop() {
        return coop;
    }
    
    public record ScoreGoal(ElevatorConstants.State state, CoralBranch branch) {}
}
