package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;
import org.steelhawks.util.LoggedTunableNumber;

@SuppressWarnings("ConstantConditions")
public class ElevatorConstants {

    public enum State {
        L4(Units.radiansToRotations(23.299634187195004)),
        L3(2.146240234375),
        L2(1.1181640625),
        L1(Units.radiansToRotations(4.947855031325136)),
        L1_JUMP(Units.radiansToRotations(4.947855031325136 + 5.0)),
        HOME(0.0),

        // move elevator up so claw is not blocking the climb and cage
        PREPARE_CLIMB(Units.radiansToRotations(10.0)),
        // This is the "HOME" position the elevator goes to, before manually going down to the ACTUAL home position at the bottom bar
        HOME_ABOVE_BAR(Units.radiansToRotations(1.0)),
        BARGE_SCORE(Units.radiansToRotations(24.0)),

        // Algae Knockout Positions
        KNOCK_L2(Units.radiansToRotations(5.0)),
        KNOCK_L3(Units.radiansToRotations(12.038681223326511));

        private final double omegaRadians;

        State(double omega) {
            this.omegaRadians = omega;
        }

        public Rotation2d getAngle() {
            return Rotation2d.fromRotations(omegaRadians);
        }
    }

    public static final int LEFT_MOTOR_ID = 13;
    public static final int RIGHT_MOTOR_ID = 14;

    public static final double MAX_ROTATIONS = Units.radiansToRotations(24.0);
    public static final double TOLERANCE = Units.radiansToRotations(0.03);
    public static final double REDUCTION = 25.0 / 1.0;
    public static final double SPROCKET_RAD = // the driving drum
        Units.inchesToMeters(1.888);

    public static final Double MAX_VELOCITY_ROT_PER_SEC = Constants.omega(25.0, 10.0);
    public static final Double MAX_ACCELERATION_ROT_PER_SEC_2 = Constants.omega(46.0, 20.0);
    public static final Double MAX_VELOCITY_ROT_PER_SEC_VOLT_PROF = Constants.omega(30.0, 10.0);
    public static final Double MAX_ACCELERATION_ROT_PER_SEC_2_VOLT_PROF = Constants.omega(30.0, 20.0);
    public static final Double MANUAL_ELEVATOR_INCREMENT = 0.65;
    public static final Double MANUAL_ELEVATOR_RAMP_RATE = Constants.omega(0.6 * MANUAL_ELEVATOR_INCREMENT, 0.8 * MANUAL_ELEVATOR_INCREMENT);

    public static final double ELEVATOR_DISTANCE_INTERPOLATOR_MAX = 0.6;
    public static final double SHIMMY_VOLTS = -5.0;
    public static final double HOMING_VOLTS = -3.0;

    public interface TorqueCurrent {
        Double[] kS = {
            Constants.omega(1.0, 0.0),
            0.0,
            0.0
        };

        Double[] kG = {
            Constants.omega(6.0, 0.0),
            0.0,
            0.0
        };

        Double[] kA = {
            0.0,
            0.0,
            0.0
        };

        LoggedTunableNumber KP = new LoggedTunableNumber("Elevator/TC/kP", Constants.omega(500.0, 6.0));
        double KI = 0.0;
        LoggedTunableNumber KD = new LoggedTunableNumber("Elevator/TC/kD", Constants.omega(40.0, 0.4));
    }

    public interface Voltage {
        Double[] kS = {
            Constants.omega(0.03, 0.0),
            0.0,
            0.0
        };

        Double[] kV = {
            Constants.omega(
                (2.0 - 1.0) / (0.6660156249999999 - 0.287109375), 0.0),
            0.0,
            0.0
        };

        Double[] kG = {
            Constants.omega(0.25, 0.0),
            0.0,
            0.0
        };

        Double[] kA = {
            0.0,
            0.0,
            0.0
        };

        LoggedTunableNumber KP = new LoggedTunableNumber("Elevator/Voltage/kP", Constants.omega(20.0, 6.0));
        double KI = 0.0;
        LoggedTunableNumber KD = new LoggedTunableNumber("Elevator/Voltage/kD", Constants.omega(5.5, 0.4));
    }
}
