package org.steelhawks.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {

    private static final List<VirtualSubsystem> mSubsystems = new ArrayList<>();
    private final String subsystemName;
    private static long lastPrintTime = 0; // Last time stats were printed
    private static long lastOverrunTime = 0; // Last time an overrun was logged

    public abstract void periodic();

    public VirtualSubsystem() {
        mSubsystems.add(this);
        subsystemName = getClass().getSimpleName();
    }

    public VirtualSubsystem(String name) {
        mSubsystems.add(this);
        subsystemName = name;
    }

    public static void periodicAll() {
        long currentTime = RobotController.getFPGATime();
        boolean shouldPrint = (currentTime - lastPrintTime) >= 10_000_000; // 10 sec
        boolean shouldPrintOverrun = (currentTime - lastOverrunTime) >= 5_000_000; // 5 sec

        StringBuilder logMessage = new StringBuilder("VirtualSubsystem Loop Times:\n");
        StringBuilder overrunMessage = new StringBuilder("WARNING: The following subsystems exceeded 20ms:\n");

        boolean overrunOccurred = false;

        for (VirtualSubsystem subsystem : mSubsystems) {
            long startTime = RobotController.getFPGATime();
            subsystem.periodic();
            long endTime = RobotController.getFPGATime();

            double loopTimeMs = (endTime - startTime) / 1_000.0; // convert to ms

            if (loopTimeMs > 20) {
                overrunOccurred = true;
                overrunMessage.append(
                    String.format("  - %s: %.3f ms\n", subsystem.subsystemName, loopTimeMs));
            }

            if (shouldPrint) {
                logMessage.append(
                    String.format("  - %s: %.3f ms\n", subsystem.subsystemName, loopTimeMs));
            }
        }

        if (overrunOccurred && shouldPrintOverrun) {
            DriverStation.reportError(overrunMessage.toString(), false);
            System.out.println("VirtualSubsystem Loop Overrun");
            lastOverrunTime = currentTime;
        }

        if (shouldPrint) {
            DriverStation.reportWarning(logMessage.toString(), false);
            lastPrintTime = currentTime;
        }
    }
}
