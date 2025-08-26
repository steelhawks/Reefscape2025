package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import org.steelhawks.util.LoggedTunableNumber;


public final class Constants {

    public static final double ENDGAME_PERIOD = 20;
    public static final double MATCH_TIME_SECONDS = 150;

    public static final int POWER_DISTRIBUTION_CAN_ID =
        getRobot() == RobotType.ALPHABOT
            ? 0
            : 1;
    public static final PowerDistribution.ModuleType PD_MODULE_TYPE =
        getRobot() == RobotType.ALPHABOT
            ? PowerDistribution.ModuleType.kCTRE
            : PowerDistribution.ModuleType.kRev;
    public static final double UPDATE_LOOP_DT = 0.020;

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public enum RobotType {
        OMEGABOT,
        ALPHABOT,
        HAWKRIDER,
        SIMBOT
    }

    // Change this based on what robot is being used.
    private static final RobotType ROBOT = RobotType.SIMBOT;

    /**
     * The robot type.
     *
     * <p>
     *     To run a physics simulator make sure you set it to RobotType.SIMBOT
     *     </p>If you want to replay a log file set it to the robot type you want to replay and just run the simulator.
     * </p>
     */
    private static final RobotType ROBOT_TYPE =
        isCI() ?
            RobotType.SIMBOT : // set to simbot when doing CI check on GitHub
            ROBOT; // actual mode you want

    private static boolean isCI() {
        return System.getenv("CI") != null;
    }

    public static final String ROBOT_NAME =
        switch (ROBOT) {
            case OMEGABOT -> "Chimera";
            case ALPHABOT -> "Alpha";
            case HAWKRIDER -> "Hawk Rider";
            case SIMBOT -> "Simulation";
        };

    public static Mode getMode() {
        return switch (ROBOT_TYPE) {
            case ALPHABOT, OMEGABOT, HAWKRIDER ->
                RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMBOT -> Mode.SIM;
        };
    }

    public static RobotType getRobot() {
        if (RobotBase.isReal() && ROBOT_TYPE == RobotType.SIMBOT) {
            new Alert("Invalid robot selected, using omega robot as default.", AlertType.kError)
                .set(true);
            return RobotType.OMEGABOT;
        }

        return ROBOT_TYPE;
    }

    public static CANBus getCANBus() {
        return switch (getRobot()) {
            case OMEGABOT, HAWKRIDER, SIMBOT -> new CANBus("canivore");
            case ALPHABOT -> new CANBus("");
        };
    }

    public static final class RobotConstants {
        public static final double BAD_BATTERY_THRESHOLD = 11.6;
        public static final double ROBOT_LENGTH_WITH_BUMPERS = Units.inchesToMeters(30.0 + (3.125 * 2.0));

        public static final double FLOOR_TO_CLAW_HEIGHT =
            Units.inchesToMeters(2.5 + 4.5 + (10.25 / 2.0) - 1.0 + 23.132);

        // for distance between robot center and claw

        // BEFORE HVR CHANGES
        // public static final double CLAW_OFFSET = -Units.inchesToMeters(9.836467);
        // public static final double CLAW_OFFSET_SMALL_COMPONENT = 0.1249231309;
        // public static final double CLAW_OFFSET_BIG_COMPONENT = 0.21637320975937756890536690206266;

        public static final double CLAW_Y_OFFSET = Units.inchesToMeters(-4.75); // -4.204645
        // AFTER HVR CHANGES. X and Y axes are based on Onshape coordinate system, NOT WPIlib coordinate system
            // This was found by taking the average of:
            // 0.649976 (the width-wise distance between the left hex shaft of the claw, and the center of the robot)
	        // 7.759314 (the width-wise distance between the right hex shaft of the claw, and the center of the robot)
//        public static final double CLAW_Y_OFFSET = Units.inchesToMeters(7.4789835);
            // This was found by taking the average of:
            // 10.783720 (the length-wise distance between the bottom front lip of the coral in the coral intake, and the center of the robot)
            // 4.174247 (the length-wise distance between the top back lip of the coral in the coral intake, and the center of the robot)

        public static final double ALGAE_CLAW_Y_OFFSET = -Units.inchesToMeters(5.081);
//        public static final double ALGAE_CLAW_Y_OFFSET = -Units.inchesToMeters(3.0);
        public static final double DIST_CLEAR_FROM_REEF = Units.inchesToMeters(13.0);

        public static final double CLIMB_Y_OFFSET = Units.inchesToMeters(12 + (3.0 / 8.0));
        public static final double DISTANCE_FROM_CAGE = Units.inchesToMeters(10.0);
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final int BUTTON_BOARD_PORT = 2;

        public static final int HOME_BUTTON_PORT = 3;
        public static final int SHOOT_BUTTON_PORT = 12;
        public static final int L1_BUTTON_PORT = 1;
        public static final int L2_BUTTON_PORT = 4;
        public static final int L3_BUTTON_PORT = 7;
        public static final int L4_BUTTON_PORT = 10;
    }

    public static final class Deadbands {
        public static final double DRIVE_DEADBAND = 0.3;
        public static final double ELEVATOR_DEADBAND = 0.05;
        public static final double PIVOT_DEADBAND = 0.1;
        public static final double REVERSE_CORAL_DEADBAND = 0.1;
        public static final double BRANCH_OVERRIDE_DEADBAND = 0.3;

        // auto align deadbands
        public static final double SWERVE_DEADBAND = 0.05;
    }

    public static final class LEDConstants {
        public static final int PORT;
        public static final int LENGTH;

        static {
            switch (getRobot()) {
                case ALPHABOT, HAWKRIDER -> {
                    PORT = 0;
                    LENGTH = 40;
                }
                default -> {
                    PORT = 0;
                    LENGTH = 80;
                }
            }
        }
    }

    @SuppressWarnings("ConstantConditions")
    public static final class AutonConstants {
        public static final LoggedTunableNumber TRANSLATION_KP = new LoggedTunableNumber("Swerve/TranslationkP", Constants.omega(5.0, 0.0));
        public static final LoggedTunableNumber TRANSLATION_KI = new LoggedTunableNumber("Swerve/TranslationkI", Constants.omega(0.0, 0.0));
        public static final LoggedTunableNumber TRANSLATION_KD = new LoggedTunableNumber("Swerve/TranslationkD", Constants.omega(0.1, 0.0));

        public static final LoggedTunableNumber ROTATION_KP = new LoggedTunableNumber("Swerve/RotationkP", Constants.omega(5.0, 0.0));
        public static final LoggedTunableNumber ROTATION_KI = new LoggedTunableNumber("Swerve/RotationkI", Constants.omega(0.0, 0.0));
        public static final LoggedTunableNumber ROTATION_KD = new LoggedTunableNumber("Swerve/RotationkD", Constants.omega(0.1, 0.0));

        private static final double MAX_TRANSLATION_VELOCITY = Constants.omega(5.0, 0.0);
        private static final double MAX_TRANSLATION_ACCELERATION = Constants.omega(6.0, 0.0);
        public static final TrapezoidProfile.Constraints ALIGN_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_TRANSLATION_VELOCITY, MAX_TRANSLATION_ACCELERATION);

        public static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("Swerve/AnglekP", Constants.omega(1.0, 2.5));
        public static final LoggedTunableNumber ANGLE_KI = new LoggedTunableNumber("Swerve/AnglekI", Constants.omega(0.0, 0.0));
        public static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("Swerve/AnglekD", Constants.omega(0.0, 1.0));

        // Pathfinder
        public static final double MAX_VELOCITY_METERS_PER_SECOND = Constants.omega(5.0, 0.0);
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = Constants.omega(5.5, 0.0);
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.omega(Units.degreesToRadians(540.0), 0.0);
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Constants.omega(Units.degreesToRadians(920.0), 0.0);

        public static final PathConstraints CONSTRAINTS = new PathConstraints(
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    /**
     * Automatically returns the correct constant based on which robot type it is running on.
     * This utility does not send the simulation value for replay mode, making replay mode work properly.
     *
     * @param hawkrider The HawkRider constant value
     * @param alpha The Alpha constant value
     * @param omega The Omega constant value
     * @param sim The simulation constant value
     * @return The correct constant
     */
    public static <T> T value(T hawkrider, T alpha, T omega, T sim) {
        return switch (Constants.getRobot()) {
            case ALPHABOT -> alpha;
            case OMEGABOT -> omega;
            case HAWKRIDER -> hawkrider;
            case SIMBOT -> sim;
        };
    }

    /**
     * Automatically returns the correct constant based on which robot type it is running on.
     * This utility does not send the simulation value for replay mode, making replay mode work properly.
     *
     * @param hawkrider The HawkRider constant value
     * @param alpha The Alpha constant value
     * @param omega The Omega constant value
     * @return The correct constant
     */
    public static <T> T value(T hawkrider, T alpha, T omega) {
        return switch (Constants.getRobot()) {
            case ALPHABOT -> alpha;
            case OMEGABOT, SIMBOT -> omega;
            case HAWKRIDER -> hawkrider;
        };
    }

    /**
     * Automatically returns the correct constant based on which robot type it is running on.
     * This utility does not send the simulation value for replay mode, making replay mode work properly.
     * In SIMBOT, the omega constant is used.
     *
     * @param alpha The Alpha constant value
     * @param omega The Omega constant value
     * @return The correct constant
     */
    public static <T> T value(T alpha, T omega) {
        return switch (Constants.getRobot()) {
            case ALPHABOT -> alpha;
            case OMEGABOT, SIMBOT -> omega;
            default -> null;
        };
    }

    /**
     * Automatically returns the correct constant based on which robot type it is running on.
     * This utility does not send the simulation value for replay mode, making replay mode work properly.
     *
     * @param omega The Omega constant value
     * @param sim The sim constant value
     * @return The correct constant
     */
    public static <T> T omega(T omega, T sim) {
        return switch (Constants.getRobot()) {
            case OMEGABOT -> omega;
            case SIMBOT -> sim;
            default -> null;
        };
    }

    public static <T> T requireNonNullConst(T obj) {
        if (obj == null) {
            DriverStation.reportWarning(
                "Robot chosen does not have this constant configured. Please null this subsystem if this was intentional.", false);
            throw new IllegalCallerException(
                "\"Robot chosen does not have this constant configured. Please null this subsystem if this was intentional.\"");
        }
        return obj;
    }
}
