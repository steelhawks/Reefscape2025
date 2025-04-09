package org.steelhawks.subsystems.algaeclaw;

import edu.wpi.first.math.geometry.Rotation2d;

public class AlgaeClawConstants {

    public enum AlgaeClawState {
        HOME(-Math.PI / 2),
        INTAKE(-Math.PI / 4),
        CATAPULT(0);

        private final double angleRads;

        AlgaeClawState(double angleRads) {
            this.angleRads = angleRads;
        }

        public Rotation2d getAngle() {
            return new Rotation2d(angleRads);
        }
    }

    public static final int PIVOT_ID = 50;
    public static final int SPIN_ID = 51;
    public static final int CANCODER_ID = 52;

    public static final double PIVOT_KS = 0.0;
    public static final double PIVOT_KG = 0.0;
    public static final double PIVOT_KV = 0.0;

    public static final double PIVOT_KP = 0.0;
    public static final double PIVOT_KI = 0.0;
    public static final double PIVOT_KD = 0.0;

    public static final double MAX_VELOCITY = 0.0;
    public static final double MAX_ACCELERATION = 0.0;
    public static final double MAX_JERK = 0.0;

    public static final double MIN_PIVOT_RADIANS = 0.0;
    public static final double MAX_PIVOT_RADIANS = 0.0;
}
