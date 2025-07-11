package org.steelhawks;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.steelhawks.subsystems.vision.VisionConstants;

import java.util.HashMap;
import java.util.Map;

public interface Toggles {

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
    }

    interface Claw {
        LoggedNetworkBoolean calculateEjectSpeed =
            new LoggedNetworkBoolean("Toggles/CalculateEjectSpeed", true);
    }
}
