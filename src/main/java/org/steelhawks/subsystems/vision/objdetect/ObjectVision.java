package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.VirtualSubsystem;

import java.util.*;
import java.util.stream.Collectors;

public class ObjectVision extends VirtualSubsystem {

    private static final double coralOverlap = 0.1; // meters
    private static final double coralMaxAge = 5.0; // seconds

    private static final LoggedTunableNumber maxArea =
        new LoggedTunableNumber("ObjectVision/MaxArea", 20.0);
    private static final LoggedTunableNumber confidenceThreshold =
        new LoggedTunableNumber("ObjectVision/ConfidenceThreshold", 0.3);

    private final ObjectVisionIO[] io;
    private final ObjectVisionIOInputsAutoLogged[] inputs;

    record CoralPose(
        Translation2d translation, double timestamp
    ) {}

    private final FieldObject2d coralObjects = FieldConstants.FIELD_2D.getObject("Corals");
    private final ArrayList<ObjectVisionIO.ObjectObservation> allObservations = new ArrayList<>();
    private final Set<CoralPose> coralPoses = new HashSet<>();

    public ObjectVision() {
        this.io = VisionConstants.getObjIO();
        this.inputs = new ObjectVisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new ObjectVisionIOInputsAutoLogged();
        }
    }

    private void addCoralObservationToPose(ObjectVisionIO.ObjectObservation observation) {
        double now = Timer.getFPGATimestamp();
        Optional<Pose2d> oldWheelOdomPose = RobotContainer.s_Swerve.getPoseAtTime(observation.timestamp());
        if (Constants.loggedValue("CoralObservation/WheelOdomEmpty", oldWheelOdomPose.isEmpty())) {
            return;
        }

        // latency compensation via interpolation
        var estimatedPose = RobotContainer.s_Swerve.getPose();
        var wheelOdometryPose = RobotContainer.s_Swerve.getWheelOdomPose();
        Pose2d fieldToRobot =
            estimatedPose.transformBy(new Transform2d(wheelOdometryPose, oldWheelOdomPose.get()));
        Transform3d robotToCamera =
            Objects.requireNonNull(VisionConstants.getObjDetectConfig())[observation.camIndex()].robotToCamera();

        // Get bounding box corners in PIXEL coordinates
        double[] txCorners = observation.info().tx();
        double[] tyCorners = observation.info().ty();

        Logger.recordOutput("CoralObservation/TxCorners", txCorners);
        Logger.recordOutput("CoralObservation/TyCorners", tyCorners);

        // Find bounding box center in pixels
        double minX = Math.min(Math.min(txCorners[0], txCorners[1]),
            Math.min(txCorners[2], txCorners[3]));
        double maxX = Math.max(Math.max(txCorners[0], txCorners[1]),
            Math.max(txCorners[2], txCorners[3]));
        double minY = Math.min(Math.min(tyCorners[0], tyCorners[1]),
            Math.min(tyCorners[2], tyCorners[3]));
        double maxY = Math.max(Math.max(tyCorners[0], tyCorners[1]),
            Math.max(tyCorners[2], tyCorners[3]));

        double centerX_pixels = (minX + maxX) / 2.0;
        double centerY_pixels = (minY + maxY) / 2.0;

        // Limelight resolution: 1280x800
        double resolutionWidth = 1280.0;
        double resolutionHeight = 800.0;

        // Convert pixels to normalized coordinates [-1, 1]
        // Center of image is at (width/2, height/2)
        double centerX = (centerX_pixels - resolutionWidth / 2.0) / (resolutionWidth / 2.0);
        double centerY = -(centerY_pixels - resolutionHeight / 2.0) / (resolutionHeight / 2.0); // negative because Y increases downward in pixels

        Logger.recordOutput("CoralObservation/centerX_pixels", centerX_pixels);
        Logger.recordOutput("CoralObservation/centerY_pixels", centerY_pixels);
        Logger.recordOutput("CoralObservation/centerX_normalized", centerX);
        Logger.recordOutput("CoralObservation/centerY_normalized", centerY);

        // Convert normalized coordinates to angles
        // Limelight 3 FOV at 1280x800: ~63.3° horizontal, ~49.7° vertical
        double horizontalFOV = 63.3; // degrees
        double verticalFOV = 49.7;   // degrees

        double tx = centerX * (horizontalFOV / 2.0); // angle in degrees
        double ty = centerY * (verticalFOV / 2.0);   // angle in degrees

        double cameraHeight = robotToCamera.getZ();
        double cameraPitch = robotToCamera.getRotation().getY(); // in radians

        // Debug logging
        Logger.recordOutput("CoralObservation/tx_degrees", tx);
        Logger.recordOutput("CoralObservation/ty_degrees", ty);
        Logger.recordOutput("CoralObservation/cameraPitch_radians", cameraPitch);

        // Convert ty from degrees to radians
        double tyRadians = Math.toRadians(ty);

        // Calculate vertical angle from horizontal
        // cameraPitch is camera tilt (positive = up), ty is offset from camera center
        double verticalAngleFromHorizontal = cameraPitch + tyRadians;

        Logger.recordOutput("CoralObservation/AngleFromHorizontal_radians", verticalAngleFromHorizontal);
        Logger.recordOutput("CoralObservation/AngleFromHorizontal_degrees", Math.toDegrees(verticalAngleFromHorizontal));

        // Target must be below horizontal to be valid
        if (Constants.loggedValue("CoralObservation/VerticalAngleError", verticalAngleFromHorizontal <= 0)) {
            return;
        }

        double targetHeight = 0.0; // coral is on the ground
        double forwardDistance = (cameraHeight - targetHeight) / Math.tan(verticalAngleFromHorizontal);

        // Calculate the translation from camera to coral in CAMERA reference frame
        double txRadians = Math.toRadians(tx);

        // Create vector from camera to coral in camera's frame
        // X-axis is forward, Y-axis is left
        Translation2d cameraToCoralInCameraFrame = new Translation2d(
            forwardDistance * Math.cos(txRadians),  // forward component
            forwardDistance * Math.sin(txRadians)   // lateral component
        );

        Logger.recordOutput("CoralObservation/cameraToCoralInCameraFrame",
            new Pose2d(cameraToCoralInCameraFrame, new Rotation2d()));

        // Transform camera pose to field coordinates
        Transform2d robotToCamera2d = new Transform2d(
            robotToCamera.getTranslation().toTranslation2d(),
            robotToCamera.getRotation().toRotation2d());
        Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera2d);

        // Debug logging
        Logger.recordOutput("CoralObservation/robotToCamera2d", robotToCamera2d);
        Logger.recordOutput("CoralObservation/cameraYaw_degrees",
            Math.toDegrees(robotToCamera.getRotation().getZ()));
        Logger.recordOutput("CoralObservation/fieldToRobot", fieldToRobot);
        Logger.recordOutput("CoralObservation/fieldToCamera", fieldToCamera);
        Logger.recordOutput("CoralObservation/forwardDistance", forwardDistance);
        Logger.recordOutput("CoralObservation/txRadians", txRadians);

        // Transform the camera-frame vector to field frame
        Pose2d fieldToCoral = fieldToCamera.transformBy(
            new Transform2d(cameraToCoralInCameraFrame, new Rotation2d())
        );

        Logger.recordOutput("CoralObservation/fieldToCoral", fieldToCoral);

        CoralPose coralPose = new CoralPose(fieldToCoral.getTranslation(), observation.timestamp());

        // Remove overlapping corals and old detections
        coralPoses.removeIf(
            c -> c.translation.getDistance(fieldToCoral.getTranslation()) <= coralOverlap
                || now - c.timestamp > coralMaxAge);
        coralPoses.add(coralPose);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < inputs.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("ObjectVision/" + io[i].getName(), inputs[i]);
            allObservations.addAll(Arrays.asList(inputs[i].observations));
        }
        allObservations.stream()
            .filter(o -> o.confidence() >= confidenceThreshold.get())
            .sorted(Comparator.comparingDouble(ObjectVisionIO.ObjectObservation::timestamp))
            .forEach(this::addCoralObservationToPose);

        coralPoses.stream()
            .forEach(o -> Logger.recordOutput("CoralDetections/Detection", new Pose2d(o.translation, new Rotation2d())));
        coralObjects.setPoses(
            coralPoses.stream()
                .map(coral -> new Pose2d(coral.translation, new Rotation2d())) // zero rotation
                .collect(Collectors.toList()));
        allObservations.clear();
    }

    public void reset() {
        coralPoses.clear();
    }
}
