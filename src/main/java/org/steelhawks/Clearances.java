package org.steelhawks;

import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.climb.Climb;
import org.steelhawks.subsystems.elevator.Elevator;

public class Clearances {

    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Claw s_Claw = RobotContainer.s_Claw;
    private static final Climb s_Climb = RobotContainer.s_Climb;

    public final static class ClawClearances {
        public static boolean hasShot = false;

        public static boolean isClearFromReef() {
            return s_Claw.hasCoral().getAsBoolean() || !hasShot;
        }
    }

    public final static class ClimbClearances {
        private static final double MIN_ANGLE_TO_CLEAR = 0.5;
        private static final double MIN_HEIGHT_TO_CLEAR = 0.5;

//        public static boolean isClearFromClaw() {
//            return s_Climb.getPosition() > MIN_ANGLE_TO_CLEAR && !RobotContainer.s_Elevator.atHome().getAsBoolean();
//        }
        public static boolean isClearFromClaw() {
            return s_Elevator.getPosition() > MIN_HEIGHT_TO_CLEAR;
        }
    }

}
