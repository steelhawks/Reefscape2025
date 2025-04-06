package org.steelhawks.util;

import java.util.function.DoubleSupplier;

/**
 * This class calculates the feedforward for an arm to overcome the acceleration due to the robot's movement.
 * It uses kG and the robot relative acceleration in Gs to compute the necessary voltage.
 *
 * @author farhanjamil
 */
public class ArmDriveFeedforward {

    private final DoubleSupplier xAccelInGs;
    private final double kG;

    /**
     * Creates a new ArmDriveFeedforward.
     *
     * @param kG the volts to hold the arm up
     * @param xAccelInGs the acceleration in Gs
     */
    public ArmDriveFeedforward(double kG, DoubleSupplier xAccelInGs) {
        this.kG = kG;
        this.xAccelInGs = xAccelInGs;
    }

    /**
     * Calculates the feedforward voltage based on the current measurement.
     *
     * @param measurement the current angle of the arm in radians
     * @return the calculated feedforward voltage
     */
    public double calculate(double measurement) {
        return -(kG * Math.sin(measurement) * xAccelInGs.getAsDouble());
    }
}
