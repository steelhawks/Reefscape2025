package org.steelhawks;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.steelhawks.subsystems.vision.VisionConstants;

import java.util.HashMap;
import java.util.Map;

public interface Toggles {

    LoggedNetworkBoolean visualizeCoralMap =
        new LoggedNetworkBoolean("Toggles/VisualizeCoralMap", false);
    LoggedNetworkBoolean autoMark =
        new LoggedNetworkBoolean("Toggles/AutoMark", true);
    LoggedNetworkBoolean debugMode =
        new LoggedNetworkBoolean("Toggles/DebugMode", false);
    LoggedNetworkBoolean tuningMode =
        new LoggedNetworkBoolean("Toggles/TuningMode", false);
    LoggedNetworkBoolean motionMagicEnabled =
        new LoggedNetworkBoolean("Toggles/MotionMagicEnabled", false);

    class Vision {
        public static final LoggedNetworkBoolean visionEnabled =
            new LoggedNetworkBoolean("Toggles/VisionEnabled", true);
        public static final Map<String, LoggedNetworkBoolean> camerasEnabled;

        static {
            camerasEnabled = new HashMap<>();
            for (String name : VisionConstants.cameraNames()) {
                camerasEnabled.put(name, new LoggedNetworkBoolean("Toggles/" + name + "Enabled", true));
            }
        }
    }

    interface Elevator {
        LoggedNetworkBoolean autoElevatorLeveling =
            new LoggedNetworkBoolean("Toggles/AutoElevatorLeveling", true);
        LoggedNetworkBoolean toggleVoltageOverride =
            new LoggedNetworkBoolean("Toggles/Elevator/VoltageOverride", false);
    }

    interface Claw {
        LoggedNetworkBoolean calculateEjectSpeed =
            new LoggedNetworkBoolean("Toggles/CalculateEjectSpeed", true);
    }

    interface AlgaeClaw {
        LoggedNetworkBoolean toggleVoltageOverride =
            new LoggedNetworkBoolean("Toggles/AlgaeClaw/VoltageOverride", false);
    }
}
