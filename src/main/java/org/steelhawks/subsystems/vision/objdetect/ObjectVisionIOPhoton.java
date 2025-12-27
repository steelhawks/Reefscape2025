package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

public class ObjectVisionIOPhoton implements ObjectVisionIO {

    private final String name;
    private final int camIndex;
    private final Transform3d robotToCamera;

    private final PhotonCamera camera;

    public ObjectVisionIOPhoton(String name, int camIndex, Transform3d robotToCamera) {
        this.name = name;
        this.camIndex = camIndex;
        this.robotToCamera = robotToCamera;

        camera = new PhotonCamera(name);
    }
}
