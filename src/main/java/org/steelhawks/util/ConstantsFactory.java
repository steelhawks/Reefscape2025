package org.steelhawks.util;

import org.steelhawks.Constants;
import org.steelhawks.Constants.RobotType;

public class ConstantsFactory {

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
}
