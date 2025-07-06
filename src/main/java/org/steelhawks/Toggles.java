package org.steelhawks;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public interface Toggles {

    interface Elevator {
        LoggedNetworkBoolean autoElevatorLeveling =
            new LoggedNetworkBoolean("Toggles/AutoElevatorLeveling", true);
    }

    interface Claw {
        LoggedNetworkBoolean calculateEjectSpeed =
            new LoggedNetworkBoolean("Toggles/CalculateEjectSpeed", true);
    }
}
