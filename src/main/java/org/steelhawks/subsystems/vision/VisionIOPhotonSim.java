package org.steelhawks.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.function.Supplier;

public class VisionIOPhotonSim extends VisionIOPhoton {

    private final VisionSystemSim mVisionSim;
    private final Supplier<Pose2d> mPoseSupplier;
    private final PhotonCameraSim cameraSim;

    public VisionIOPhotonSim(
        String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.mPoseSupplier = poseSupplier;

        mVisionSim = new VisionSystemSim("main");
        mVisionSim.addAprilTags(KVision.APRIL_TAG_FIELD_LAYOUT);

        // add cam to sim
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(mCamera, cameraProperties);
        mVisionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        mVisionSim.update(mPoseSupplier.get());
        super.updateInputs(inputs);
    }
}
