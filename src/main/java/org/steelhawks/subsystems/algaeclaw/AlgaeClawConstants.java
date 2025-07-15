package org.steelhawks.subsystems.algaeclaw;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.util.LoggedTunableNumber;

public class AlgaeClawConstants {

    public enum State {
        HOME(-1.5),
        AVOID(-1.0),
        PROCESSOR(-0.8),
        PARALLEL(0.0),
        INTAKE(-Math.PI / 4),
        CATAPULT(0.6);

        private final double angleRads;

        State(double angleRads) {
            this.angleRads = angleRads;
        }

        public Rotation2d getAngle() {
            return new Rotation2d(angleRads);
        }
    }

    public static final CANBus CLAW_BUS = new CANBus();

    /* ------------- Simulation ------------- */
    public static final double GEAR_RATIO = 100.0;
    public static final double ARM_LENGTH = Units.inchesToMeters(8.0);
    public static final double ARM_MASS_KG = Units.lbsToKilograms(12.0);
    public static final double MOMENT_OF_INERTIA = ARM_MASS_KG * Math.pow(ARM_LENGTH, 2) / 3.0;

    public static final int PIVOT_ID = 25;
    public static final int SPIN_ID = 26;
    public static final int CANCODER_ID = 27;

    public static final LoggedTunableNumber PIVOT_KS = new LoggedTunableNumber("AlgaeClaw/kS",0.0);
    public static final LoggedTunableNumber PIVOT_KG = new LoggedTunableNumber("AlgaeClaw/kG",0.0);
    public static final LoggedTunableNumber PIVOT_KV = new LoggedTunableNumber("AlgaeClaw/kV",0.0);
    public static final LoggedTunableNumber PIVOT_KA = new LoggedTunableNumber("AlgaeClaw/kA",0.0);

    public static final LoggedTunableNumber PIVOT_KP = new LoggedTunableNumber("AlgaeClaw/kP",0.0);
    public static final LoggedTunableNumber PIVOT_KI = new LoggedTunableNumber("AlgaeClaw/kI",0.0);
    public static final LoggedTunableNumber PIVOT_KD = new LoggedTunableNumber("AlgaeClaw/kD",0.0);

    public static final double MAX_VELOCITY_RAD_PER_SEC = 0.0;
    public static final double MAX_ACCELERATION_RAD_PER_SEC_2 = 0.0;
    public static final double MAX_MANUAL_SPEED = 0.3;
    public static final double TOLERANCE = 0.02;
    public static final Rotation2d CANCODER_OFFSET = Rotation2d.fromRotations(0.0); // in rotations
    public static final double MIN_PIVOT_RADIANS = -Math.PI / 2.0;
    public static final double MAX_PIVOT_RADIANS = 0.6504078540635119;

    public static final double INTAKE_SPEED = 1.0;
    public static final double RETAIN_ALGAE_SPEED = 0.5;
    public static final double CURRENT_THRESHOLD_TO_HAVE_ALGAE = 80;
}
