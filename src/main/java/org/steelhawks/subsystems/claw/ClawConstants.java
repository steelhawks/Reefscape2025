package org.steelhawks.subsystems.claw;

import org.steelhawks.Constants;

public class ClawConstants {

    public static final int CAN_RANGE_ID_OMEGA = 17;

    public static final int CLAW_INTAKE_MOTOR_ID;

    public static final double CLAW_INTAKE_GEAR_RATIO;

    public static final double CLAW_SHOOT_SPEED;
    public static final double CLAW_SECONDARY_SHOOT_SPEED;
    public static final double CLAW_TERTIARY_SHOOT_SPEED;
    public static final double CLAW_INTAKE_SPEED;

    static {
        switch (Constants.getRobot()) {
            case ALPHABOT, HAWKRIDER -> {
                CLAW_INTAKE_MOTOR_ID = 16;
                CLAW_INTAKE_GEAR_RATIO = 1.0;
                CLAW_SHOOT_SPEED = 0.1;
                CLAW_SECONDARY_SHOOT_SPEED = 0.135;
                CLAW_TERTIARY_SHOOT_SPEED = 0.05;
                CLAW_INTAKE_SPEED = 0.1;
            }
            default -> {
                CLAW_INTAKE_MOTOR_ID = 15;
                CLAW_INTAKE_GEAR_RATIO = 1.0;
                CLAW_SHOOT_SPEED = 0.25;
                CLAW_SECONDARY_SHOOT_SPEED = 0.225;
                CLAW_TERTIARY_SHOOT_SPEED = 0.05;
                CLAW_INTAKE_SPEED = 0.1;
            }
        }
    }
}
