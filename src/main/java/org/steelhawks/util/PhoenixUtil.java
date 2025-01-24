package org.steelhawks.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

public class PhoenixUtil {
    /**
     * Attempts to run the command until no error is produced.
     */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }

    /**
     * Used for MapleSim simulation for the Gyro and Odometry
     */
    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] = Timer.getFPGATimestamp()
                - 0.02
                + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }
}
