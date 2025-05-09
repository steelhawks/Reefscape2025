package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
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

    private static final Map<String, String> BRANCH_CODE = Map.ofEntries(
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
    private Map<String, Boolean> algaeMap;
    private int troughCount;
    private boolean coop;

    public ReefState() {
        coralMap = new HashMap<>();
        for (String name : REEF_NAMES) {
            coralMap.put(name, new boolean[3]);
        }
        troughCount = 0;
        coop = false;

        algaeMap = new HashMap<>();
        for (String code : new String[]{"L", "TL", "TR", "R", "BR", "BL"}) {
            algaeMap.put(code, false);
        }
    }

    @Override
    public void periodic() {
        updateFromNetworkTables();
        syncVisualizer(); // push our boolean‑map into the visualizer
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

        // 2) re‑score every coral toggle for levels2–4
        for (var entry : coralMap.entrySet()) {
            String reefName = entry.getKey();
            boolean[] levels = entry.getValue();
            for (int lvlIdx = 0; lvlIdx < levels.length; lvlIdx++) {
                if (levels[lvlIdx]) {
                    // map lvlIdx to dashboard level: 0→4, 1→3, 2→2
                    int dashLevel = (lvlIdx == 0 ? 4 : lvlIdx == 1 ? 3 : 2);
                    // reefName is e.g. "leftOne", prepend "L4_", "L3_" or "L2_"
                    String enumName = "L" + dashLevel + "_" + toBranchCode(reefName);
                    ReefVisualizer.scoreCoral(enumName);
                }
            }
        }

        // 3) clear all algae, then re‑add only those not removed
        for (String code : algaeMap.keySet()) {
            ReefVisualizer.removeAlgae(code);
        }
        for (Map.Entry<String, Boolean> e : algaeMap.entrySet()) {
            String code = e.getKey();
            boolean removed = e.getValue();
            if (!removed) {
                // algae still present, add it
                ReefVisualizer.addAlgae(code);
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

        for (String code : algaeMap.keySet()) {
            boolean removed = table.getEntry("algae_" + code).getBoolean(false);
            algaeMap.put(code, removed);
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
            .getEntry(troughKey)
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
     * “minimum” is 7.
     */
    public boolean achievedLevelMinimum(int level) {
        int needed = 7;
        return getCountForLevel(level) >= needed;
    }

    public boolean achievedCoralRP() {
        return NetworkTableInstance.getDefault().getTable("ReefData").getEntry(coopKey).getBoolean(false);
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
     * Find the next best branch to score:
     * highest reef‑level first (L4→L3→L2)
     * then minimal travel distance
     */
    public ScoreGoal getNextBestScorePosition() {
        Pose2d robotPose = RobotContainer.s_Swerve.getPose();

        ScoreGoal best = null;
        int bestLevelPrio = Integer.MAX_VALUE;
        double bestDist = Double.MAX_VALUE;

        for (String reefName : REEF_NAMES) {
            boolean[] arr = coralMap.get(reefName);
            for (int idx = 0; idx < arr.length; idx++) {
                if (!arr[idx]) {
                    // idx 0→L4, 1→L3, 2→L2
                    int levelPrio = idx;
                    ElevatorConstants.State state =
                        (idx == 0 ? ElevatorConstants.State.L4
                            : idx == 1 ? ElevatorConstants.State.L3
                            : ElevatorConstants.State.L2);

                    // branch code "L1","TL2", etc.
                    String code = toBranchCode(reefName);
                    CoralBranch branch = CoralBranch.valueOf(code);

                    // distance from robot to that branch
                    Pose2d branchPose = branch.getBranchPoseProjectedToReefFace();
                    double dist = robotPose.getTranslation()
                        .getDistance(branchPose.getTranslation());

                    // choose if better level, or same level but closer
                    if (levelPrio < bestLevelPrio ||
                        (levelPrio == bestLevelPrio && dist < bestDist)) {
                        bestLevelPrio = levelPrio;
                        bestDist = dist;
                        best = new ScoreGoal(state, branch);
                    }
                }
            }
        }

        // null if everything already scored, continues to l1 scoring only
        return best != null ? best : new ScoreGoal(ElevatorConstants.State.L1, ReefUtil.getClosestCoralBranch());
    }

    // create an algorithm to achieve coral rp the fastest
    // make sure it returns to maximize points algorithm after it achieves rp
    /**
     * Pick the next score action (either a coral branch or a trough increment)
     * that most advances you toward CoralRP (7 on each level, or 7 on any 3 levels if coop).
     */
    public ScoreGoal getNextForCoralRP() {
        Pose2d robotPose = RobotContainer.s_Swerve.getPose();

        // 1) find which level we need to score next to hit the RP thresholds
        int consecutiveFull = 0;
        int targetLevel = 1;
        for (int lvl = 4; lvl >= 1; lvl--) {
            if (achievedLevelMinimum(lvl)) {
                consecutiveFull++;
                // once 3 higher levels are full, switch to max‑points
                if (consecutiveFull == 3) {
                    return getNextBestScorePosition();
                }
            } else {
                targetLevel = lvl;
                break;
            }
        }

        // map targetLevel, array index (0→L4, 1→L3, 2→L2, 3→L1)
        int idxWanted = 4 - targetLevel;

        ElevatorConstants.State targetState =
            switch (targetLevel) {
                case 4 -> ElevatorConstants.State.L4;
                case 3 -> ElevatorConstants.State.L3;
                case 2 -> ElevatorConstants.State.L2;
                default -> ElevatorConstants.State.L1;
            };

        // 2) among all reefs, choose the closest branch at that level
        double bestDist = Double.MAX_VALUE;
        ScoreGoal bestGoal = null;

        for (String reefName : REEF_NAMES) {
            boolean[] arr = coralMap.get(reefName);
            // skip if already scored at this level
            if (idxWanted < 0 || idxWanted >= arr.length || arr[idxWanted]) {
                continue;
            }

            // build branch enum
            String code = toBranchCode(reefName);
            CoralBranch branch = CoralBranch.valueOf(code);

            // distance from robot to branch
            Pose2d branchPose = branch.getBranchPoseProjectedToReefFace();
            double dist = robotPose.getTranslation().getDistance(branchPose.getTranslation());

            if (dist < bestDist) {
                bestDist = dist;
                bestGoal = new ScoreGoal(targetState, branch);
            }
        }

        // if nothing found, fallback to max‑points
        return (bestGoal != null ? bestGoal : getNextBestScorePosition());
    }

    public ScoreGoal dynamicScoreRoutine() {
        return achievedCoralRP() ? getNextBestScorePosition() : getNextForCoralRP();
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
