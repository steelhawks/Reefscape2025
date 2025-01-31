package org.steelhawks.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

public class PhoenixUtil {

    private static double lastStateChangeTime = Timer.getFPGATimestamp();
    private static boolean lastSwitchState = false;

    /**
     * Attempts to check if a DigitalInput is connected by checking if the state has changed.
     *
     * @param timeoutSecs the time in seconds to wait for a state change before assuming the DigitalInput is disconnected
     * @param mLimitSwitch the DigitalInput to check
     */
    public static boolean digitalInputAlive(int timeoutSecs, DigitalInput mLimitSwitch) {
        boolean currentState = mLimitSwitch.get();
        boolean changed = (currentState != lastSwitchState);

        if (changed) {
            lastSwitchState = currentState;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }

        // if no change assume disconnected
        return changed || (Timer.getFPGATimestamp() - lastStateChangeTime < timeoutSecs);
    }

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
