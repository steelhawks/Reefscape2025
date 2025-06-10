package org.steelhawks.util;

import org.steelhawks.Constants;
import org.steelhawks.Constants.RobotType;

public class ConstantsFactory {
    /**
     * Automatically returns the correct constant based on if running in a simulation vs. replay or on real.
     * This utility does not send the simulation value for replay mode, making replay mode work properly.
     *
     * @param real The real constant value
     * @param sim The simulation constant value
     * @return The correct constant
     */
    public static double value(double real, double sim) {
        return Constants.getRobot() == RobotType.SIMBOT
            ? sim
            : real;
    }
}
