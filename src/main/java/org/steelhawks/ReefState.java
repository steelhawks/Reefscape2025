package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.ReefUtil.CoralBranch;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.LoopTimeUtil;
import org.steelhawks.util.VirtualSubsystem;
import java.util.*;

import static edu.wpi.first.networktables.NetworkTableEvent.Kind.*;

public class ReefState extends VirtualSubsystem {

    private static final List<ScoreGoal> scoredGoals = new ArrayList<>();
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

    private static String toBranchCode(String reefName) {
        return BRANCH_CODE.getOrDefault(reefName, reefName);
    }

    private CoralBranch toCodeBranch(String code) {
        return CoralBranch.valueOf(code);
    }

    private static String toBranchFromCode(String code) {
        for (Map.Entry<String, String> entry : BRANCH_CODE.entrySet()) {
            if (entry.getValue().equals(code)) {
                return entry.getKey();
            }
        }
        return code;
    }

    private static String toCodeFromBranch(CoralBranch branch) {
        return toBranchFromCode(branch.name());
    }

    private static final NetworkTableEntry troughEntry;
    private static final NetworkTableEntry coopEntry;
    private static final NetworkTableEntry overrideEntry;
    private static final NetworkTableEntry goalEntry;

    // for levels 2–4 each reef has 3 branches; level1 uses troughCount
    private static final NetworkTable reefData =
        NetworkTableInstance.getDefault().getTable("ReefData");
    private static final Map<String, boolean[]> coralMap;
    private static final Map<String, Boolean> algaeMap;
    private static final Map<String, NetworkTableEntry> entryMap;
    private static int troughCount;
    private static boolean coop;
    private int counter = 0;

    static {
        troughEntry = reefData.getEntry("TroughCount");
        coopEntry = reefData.getEntry("coop");
        overrideEntry = reefData.getEntry("override");
        goalEntry = reefData.getEntry("goal");

        coralMap = new HashMap<>();
        for (String name : REEF_NAMES) {
            coralMap.put(name, new boolean[3]);
        }
        algaeMap = new HashMap<>();
        for (String code : new String[]{"L", "TL", "TR", "R", "BR", "BL"}) {
            algaeMap.put(code, false);
        }
        entryMap = new HashMap<>();
        for (String name : REEF_NAMES) {
            for (int idx = 0; idx < 3; idx++) {
                String key = name + "_" + idx;
                entryMap.put(key, reefData.getEntry(key));
            }
        }
        for (String code : algaeMap.keySet()) {
            String key = "algae_" + code;
            entryMap.put(key, reefData.getEntry(key));
        }
        entryMap.put("TroughCount", troughEntry);
        entryMap.put("coop", coopEntry);
        entryMap.put("override", overrideEntry);
        entryMap.put("goal", goalEntry);
        new ReefState();
    }

    public ReefState() {
        troughCount = 0;
        coop = false;

        updateFromNetworkTables(); // update existing data before first callback
        reefData.addListener(
            // WARNING!!!
            // DO NOT LOG HERE, AdvantageKit is not thread-safe and you will ruin replay
            // https://docs.advantagekit.org/getting-started/common-issues/multithreading
            EnumSet.of(kImmediate, kValueLocal, kValueRemote),
            (NetworkTable table, String key, NetworkTableEvent event) -> {
                NetworkTableEntry entry = table.getEntry(key);
                if (key.equals("TroughCount")) {
                    troughCount = (int) entry.getInteger(0);
                }
                else if (key.equals("coop")) {
                    coop = entry.getBoolean(false);
                }
                else if (key.startsWith("algae_")) {
                    String code = key.substring("algae_".length());
                    algaeMap.put(code, entry.getBoolean(false));
                }
                // coral toggles only on the keys that have "_"
                else if (key.contains("_")) {
                    String[] parts = key.split("_", 2);
                    String reefName = parts[0];
                    int idx = Integer.parseInt(parts[1]);
                    boolean val= entry.getBoolean(false);

                    // check if u have weird keys
                    if (coralMap.containsKey(reefName) && idx >= 0 && idx < 3) {
                        coralMap.get(reefName)[idx] = val;
                    }
                }
            }
        );
    }

    @Override
    public void periodic() {
        // update from the main thread because the listeners are not thread-safe for akit.
        // anything that needs to be logged using recordOutput or processInputs that comes
        // from NT should be logged here and handled in the thread by storing it in an object
        // and then passing it through here to log
        // https://docs.advantagekit.org/getting-started/common-issues/multithreading
        final boolean shouldRun =
            counter >= 50 // 50 * 20ms = 1 second
                && !Robot.getState().equals(Robot.RobotState.AUTON)
                && !DriverStation.isFMSAttached()
                && !DriverStation.isDisabled()
            && Toggles.visualizeCoralMap.get();
        counter++;
        if (shouldRun) {
            syncVisualizer();
            ReefVisualizer.updateVisualizer();
            LoopTimeUtil.record("ReefState/VisualizerUpdate");
            counter = 0;
        }
        Logger.recordOutput("ReefState/ShouldRun", shouldRun);
        LoopTimeUtil.record("ReefState");
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
                    // map lvlIdx to dashboard level: 0->4, 1->3, 2->2
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
        troughCount = (int) entryMap.get("TroughCount").getInteger(0);
        coop = entryMap.get("coop").getBoolean(false);

        for (String name : REEF_NAMES) {
            boolean[] arr = coralMap.get(name);
            for (int idx = 0; idx < arr.length; idx++) {
                arr[idx] = entryMap.get(name + "_" + idx).getBoolean(false);
            }
        }

        for (String code : algaeMap.keySet()) {
            boolean removed = entryMap.get("algae_" + code).getBoolean(false);
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
//        reefData.getEntry(reefName + "_" + levelIndex).setBoolean(true);
        entryMap.get(reefName + "_" + levelIndex).setBoolean(true);
    }

    /**
     * Called from robot code when you score a coral.
     */
    public static void scoreCoral(CoralBranch branch, ElevatorConstants.State level) {
        int index =
            level == ElevatorConstants.State.L4
                ? 0 : level == ElevatorConstants.State.L3
                ? 1 : 2;
        coralMap.get(toCodeFromBranch(branch))[index] = true;
        scoredGoals.add(new ScoreGoal(level, branch));

//        reefData.getEntry(toCodeFromBranch(branch) + "_" + index).setBoolean(true);
        entryMap.get(toCodeFromBranch(branch) + "_" + index).setBoolean(true);
    }

    public static void removeScoredCoral(ScoreGoal goal) {
        removeScoredCoral(goal.branch, goal.state);
    }

    public static void removeScoredCoral(CoralBranch branch, ElevatorConstants.State level) {
        int index = switch (level) {
            case L4 -> 0;
            case L3 -> 1;
            case L2 -> 2;
            default -> -1;
        };
        if (index < 0) return;

        String key = toCodeFromBranch(branch);
        boolean[] levels = coralMap.get(key);
        if (levels == null) return;

        levels[index] = false;

        entryMap.get(key + "_" + index).setBoolean(false);

        for (int i = scoredGoals.size() - 1; i >= 0; i--) {
            ScoreGoal goal = scoredGoals.get(i);
            if (goal.state() == level && goal.branch() == branch) {
                scoredGoals.remove(i);
                break;
            }
        }
    }


    /**
     * Called when you change troughCount (level1).
     */
    public static void setTroughCount(int count) {
        troughCount = count;
        entryMap.get("TroughCount").setNumber(count);
    }

    /**
     * Called when coop toggles.
     */
    public static void setCoop(boolean c) {
        coop = c;
        entryMap.get("coop").setBoolean(c);
    }

    /**
     * Returns total coral‑toggles at dashboard level (4->index0, 3->1, 2->2, 1->troughCount).
     */
    public static int getCountForLevel(int level) {
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
     * “minimum" is 7.
     */
    public static boolean achievedLevelMinimum(int level) {
        int needed = 7;
        return getCountForLevel(level) >= needed;
    }

    /**
     * Coral RP: either 4 levels ≥7 (no‑coop) or 3 levels ≥7 + coop.
     */
    public static boolean achievedCoralRP() {
        int qualified = 0;
        for (int lvl = 2; lvl <= 4; lvl++) {
            if (getCountForLevel(lvl) >= 7) qualified++;
        }
        return coop ? (qualified >= 3) : (qualified == 3);
    }

    /**
     * Find the next best branch to score:
     * highest reef‑level first (L4->L3->L2)
     * then minimal travel distance
     */
    public static ScoreGoal getNextBestScorePosition() {
        Pose2d robotPose = RobotContainer.s_Swerve.getPose();

        ScoreGoal best = null;
        int bestLevelPrio = Integer.MAX_VALUE;
        double bestDist = Double.MAX_VALUE;

        for (String reefName : REEF_NAMES) {
            boolean[] arr = coralMap.get(reefName);
            for (int idx = 0; idx < arr.length; idx++) {
                if (!arr[idx]) {
                    // idx 0->L4, 1->L3, 2->L2
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
    public static ScoreGoal getNextForCoralRP() {
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

        // map targetLevel, array index (0->L4, 1->L3, 2->L2, 3->L1)
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

    public static ScoreGoal getQuickestScoring() {
        Pose2d robotPose = RobotContainer.s_Swerve.getPose();

        ScoreGoal best = null;
        double bestDist = Double.MAX_VALUE;
        int bestLevelPrio = Integer.MAX_VALUE;

        for (String reefName : REEF_NAMES) {
            boolean[] arr = coralMap.get(reefName);
            for (int idx = 0; idx < arr.length; idx++) {
                if (!arr[idx]) {
                    // idx 0->L4, 1->L3, 2->L2
                    ElevatorConstants.State state =
                        (idx == 0 ? ElevatorConstants.State.L4
                            : idx == 1 ? ElevatorConstants.State.L3
                            : ElevatorConstants.State.L2);

                    CoralBranch branch = CoralBranch.valueOf(toBranchCode(reefName));
                    double dist = robotPose.getTranslation()
                        .getDistance(branch.getBranchPoseProjectedToReefFace().getTranslation());

                    // now compare: closest first, then higher level
                    if (dist < bestDist ||
                        (dist == bestDist && idx < bestLevelPrio)) {
                        bestDist = dist;
                        bestLevelPrio = idx;
                        best = new ScoreGoal(state, branch);
                    }
                }
            }
        }

        // if everything’s scored, fall back to L1 on the absolute closest branch
        return best != null
            ? best
            : new ScoreGoal(ElevatorConstants.State.L1, ReefUtil.getClosestCoralBranch());
    }

    public static CoralBranch getFreeBranch(ElevatorConstants.State targetState) {
        Pose2d robotPose = RobotContainer.s_Swerve.getPose();
        // map targetLevel, array index (0->L4, 1->L3, 2->L2, 3->L1)
        int idxWanted =
            switch (targetState) {
                case L4 -> 0;
                case L3 -> 1;
                case L2 -> 2;
                default -> -1;
            };

        // 2) among all reefs, choose the closest branch at that level
        double bestDist = Double.MAX_VALUE;
        CoralBranch bestBranch = null;

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
                bestBranch = branch;
            }
        }

        return bestBranch != null ? bestBranch : ReefUtil.getClosestCoralBranch();
    }

    public static ScoreGoal dynamicScoreRoutine() {
        String goal = entryMap.get("goal").getString("");
        Logger.recordOutput("Align/AutoScoreGoal", goal);
        return switch (goal) {
            case "CORALRP" -> getNextForCoralRP();
            case "FASTEST" -> getQuickestScoring();
            default -> getNextBestScorePosition();
        };
    }

    // getters for dashboard
    public static Map<String, boolean[]> getAllCoralMaps() {
        return coralMap;
    }

    public static ScoreGoal getLastScoredPosition() {
        if (scoredGoals.isEmpty()) {
            return null;
        }
        return scoredGoals.get(scoredGoals.size() - 1);
    }

    public static int getTroughCount() {
        return troughCount;
    }

    public static boolean isCoop() {
        return coop;
    }

    public static boolean hasOverriden() {
        return entryMap.get("override").getBoolean(false);
    }

    public record ScoreGoal(ElevatorConstants.State state, CoralBranch branch) {}
}
