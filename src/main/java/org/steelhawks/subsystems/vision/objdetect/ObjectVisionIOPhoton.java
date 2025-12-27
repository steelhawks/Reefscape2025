package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.math.geometry.Transform3d;

public class ObjectVisionIOPhoton implements ObjectVisionIO {

    private final String name;
    private final Transform3d robotToCamera;

    public ObjectVisionIOPhoton(String name, Transform3d robotToCamera) {
        this.name = name;
        this.robotToCamera = robotToCamera;
    }
}
