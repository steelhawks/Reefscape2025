package org.steelhawks;

import org.steelhawks.subsystems.algaeclaw.AlgaeClaw;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.elevator.Elevator;

public class Clearances {

    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Claw s_Claw = RobotContainer.s_Claw;
    private static final AlgaeClaw s_AlgaeClaw = RobotContainer.s_AlgaeClaw;

    public final static class ClawClearances {
        public static boolean hasShot = false;

        public static boolean isClearFromReef() {
            return s_Claw.hasCoral().getAsBoolean() || !hasShot;
        }
    }

    public final static class AlgaeClawClearances {
        private static final double MIN_ANGLE_CLEAR_FROM_HOME = -1.1274758790959463;
        private static final double ELEVATOR_COLLIDE_ANGLE = -0.5;
        private static final double[] COLLISION_INTERVAL =
            new double[] {
                20, 30
            };

        public static boolean willCollideIntoElevator() {
            return s_AlgaeClaw.getPivotPosition() < MIN_ANGLE_CLEAR_FROM_HOME && s_Elevator.getPosition() < ELEVATOR_COLLIDE_ANGLE;
        }

        public static boolean safeToGoHome() {
            return s_AlgaeClaw.getPivotPosition() > COLLISION_INTERVAL[0] && s_AlgaeClaw.getPivotPosition() < COLLISION_INTERVAL[1];
        }

        public static boolean isClearFromElevatorCrossbeam() {
            return s_AlgaeClaw.getPivotPosition() >= MIN_ANGLE_CLEAR_FROM_HOME;
        }
    }

    public final static class ClimbClearances {
        private static final double MIN_HEIGHT_TO_CLEAR = 1.1259418983080607;

        public static boolean isClearFromClaw() {
            return s_Elevator.getPosition() >= MIN_HEIGHT_TO_CLEAR;
        }
    }

}
