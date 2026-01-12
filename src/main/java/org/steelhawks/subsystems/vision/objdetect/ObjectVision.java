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
        // take the estimated current robot pose and offset it by how the
        // robot actually moved according to the wheel odometry since the observation
        Pose2d fieldToRobot =
            estimatedPose.transformBy(new Transform2d(wheelOdometryPose, oldWheelOdomPose.get()));
        Transform3d robotToCamera =
            Objects.requireNonNull(VisionConstants.getObjDetectConfig())[observation.camIndex()].robotToCamera();

        // find object midpoint OLD WAY ONLY WORKS FOR LIMELIGHT
//        double tx = (observation.info().tx()[2] + observation.info().tx()[3]) / 2.0;
//        double ty = (observation.info().ty()[2] + observation.info().ty()[3]) / 2.0;

        // bounding box center, works for both limelight and photon,
        // where photon doesnt guarantee the same corner layout, no particular order
        double[] txs = observation.info().tx();
        double[] tys = observation.info().ty();

        double minX = Arrays.stream(txs).min().orElse(0.0);
        double maxX = Arrays.stream(txs).max().orElse(0.0);
        double minY = Arrays.stream(tys).min().orElse(0.0);
        double maxY = Arrays.stream(tys).max().orElse(0.0);

        double tx = (minX + maxX) / 2.0;
        double ty = (minY + maxY) / 2.0;

        double cameraHeight = robotToCamera.getZ();
        double cameraPitch = robotToCamera.getRotation().getY();

        // vert height from camera to target
        double verticalAngleFromHorizontal = cameraPitch - ty; // apparently cameraPitch should not be negated
        if (Constants.loggedValue("CoralObservation/VerticalAngleError", verticalAngleFromHorizontal <= 0)) { // target above horizontal is invalid
            return;
        }
        double targetHeight = 0.0; // coral is on the ground so target height should be 0.0
        double forwardDistance = (cameraHeight - targetHeight) / Math.tan(verticalAngleFromHorizontal); // distance along camera forward axis
        double lateralCorrection = 1.0 / Math.cos(-tx); // correction for horizontal angle to get lateral distance
        double cameraToObjectNorm = forwardDistance * lateralCorrection;

        Transform2d robotToCamera2d =
            new Transform2d(
                robotToCamera.getTranslation().toTranslation2d(),
                robotToCamera.getRotation().toRotation2d());
        Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera2d);
        Pose2d fieldToCoral =
            fieldToCamera
                .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-tx)))
                .transformBy(
                    new Transform2d(new Translation2d(cameraToObjectNorm, 0), Rotation2d.kZero));
        CoralPose coralPose = new CoralPose(fieldToCoral.getTranslation(), observation.timestamp());
        // delete coral once close enough to robot and deletes coral that has existed longer than 10 seconds
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
