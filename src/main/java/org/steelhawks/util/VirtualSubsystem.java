package org.steelhawks.util;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {

    private static List<VirtualSubsystem> mSubsystems = new ArrayList<>();
    public abstract void periodic();

    public VirtualSubsystem() {
        mSubsystems.add(this);
    }

    public static void periodicAll() {
        for (VirtualSubsystem subsystem : mSubsystems) {
            subsystem.periodic();
        }
    }
}
