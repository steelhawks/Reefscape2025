package org.steelhawks.subsystems.algaeclaw;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlgaeClawConstants {

    public enum AlgaeClawState {
        HOME(-1.5),
        INTAKE(-Math.PI / 4),
        CATAPULT(0.6);

        private final double angleRads;

        AlgaeClawState(double angleRads) {
            this.angleRads = angleRads;
        }

        public Rotation2d getAngle() {
            return new Rotation2d(angleRads);
        }
    }

    public static final CANBus CLAW_BUS = new CANBus();

    public static final int PIVOT_ID = 25;
    public static final int SPIN_ID = 26;
    public static final int CANCODER_ID = 27;

    public static final double PIVOT_KS = 0.15;
    public static final double PIVOT_KG = 0.0;
    public static final double PIVOT_KV = 0.0;

    public static final double PIVOT_KP = 0.0;
    public static final double PIVOT_KI = 0.0;
    public static final double PIVOT_KD = 0.0;

    public static final double MAX_VELOCITY = 0.0;
    public static final double MAX_ACCELERATION = 0.0;
    public static final double MAX_JERK = 0.0;

    public static final double CANCODER_OFFSET = -0.1103515625;
    public static final double MIN_PIVOT_RADIANS = -1.5631264228554684;
    public static final double MAX_PIVOT_RADIANS = 0.6504078540635119;
}
