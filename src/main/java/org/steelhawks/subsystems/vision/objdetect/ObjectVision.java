package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.*;
import java.util.stream.Collectors;

public class ObjectVision extends SubsystemBase {

    private static final double coralOverlap = 0.5; // meters
    private static final double coralMaxAge = 10.0; // seconds

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

    /*
     * NEED TO TUNE MAX AREA
     * NEED TO TUNE W1, W2, W3
     * Log values to tune properly
     */
    public static double calcConfidence(ObjectVisionIO.DetectionInfo d, int cameraIndex) {
        String logName = "ObjectVision/" +
            Objects.requireNonNull(VisionConstants.getObjDetectConfig())[cameraIndex].name() + "/";

        // areaScore: normalized 0–1
        double areaScore = Math.min(d.area() / maxArea.get(), 1.0);

        // shapeScore: 0 = skewed, 1 = rectangle
        double width = Math.abs(d.tx()[0] - d.tx()[1]);
        double height = Math.abs(d.ty()[0] - d.ty()[2]);
        double diag1 = Math.hypot(d.tx()[0] - d.tx()[2], d.ty()[0] - d.ty()[2]);
        double diag2 = Math.hypot(d.tx()[1] - d.tx()[3], d.ty()[1] - d.ty()[3]);
        double ratio = Math.min(diag1, diag2) / Math.max(diag1, diag2); // 0–1, shapeScore

        // angleScore: penalize extreme horizontal angles
        double txAvg = (d.tx()[0] + d.tx()[1] + d.tx()[2] + d.tx()[3]) / 4.0;
        double angleScore = Math.cos(txAvg); // roughly favors smaller offsets

        double w1 = 0.5; // how much the area matters to confidence
        double w2 = 0.3; // how much the shape matters to confidence
        double w3 = 0.2; // how much the angle matters to confidence

        return MathUtil.clamp(Constants.loggedValue(logName + "w1", w1 * areaScore) +
            Constants.loggedValue(logName + "w2", w2 * ratio) +
                Constants.loggedValue(logName + "w3", w3 * angleScore), 0.0, 1.0);
    }


    private void addCoralObservationToPose(ObjectVisionIO.ObjectObservation observation) {
        double now = Timer.getFPGATimestamp();
        Optional<Pose2d> oldWheelOdomPose = RobotContainer.s_Swerve.getPoseAtTime(observation.timestamp());
        if (oldWheelOdomPose.isEmpty()) {
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
        double verticalAngle = -cameraPitch - ty;
        if (verticalAngle <= 0) { // target above horizontal is invalid
            return;
        }
        double targetHeight = 0.0; // coral is on the ground so target height should be 0.0
        double forwardDistance = (cameraHeight - targetHeight) / Math.tan(verticalAngle); // distance along camera forward axis
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
            .filter(o -> (Objects.requireNonNull(VisionConstants.getObjDetectConfig())[o.camIndex()]
                .cameraType().equals(VisionConstants.CameraConfig.CameraType.LIMELIGHT)
                    ? calcConfidence(o.info(), o.camIndex()) : o.confidence()) >= confidenceThreshold.get()
            )
            .sorted(Comparator.comparingDouble(ObjectVisionIO.ObjectObservation::timestamp))
            .forEach(this::addCoralObservationToPose);

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
