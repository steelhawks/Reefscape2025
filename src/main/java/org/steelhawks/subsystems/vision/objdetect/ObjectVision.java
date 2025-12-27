package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.vision.VisionConstants;

import java.util.*;
import java.util.stream.Collectors;

public class ObjectVision extends SubsystemBase {

    private static final double coralOverlap = 0.5; // meters
    private static final double coralMaxAge = 10.0; // seconds

    private final ObjectVisionIO[] io;
    private final ObjectVisionIOInputsAutoLogged[] inputs;

    record CoralPose(
        Translation2d translation, double timestamp
    ) {}

    private final FieldObject2d coralObjects = FieldConstants.FIELD_2D.getObject("Corals");
    private final ArrayList<ObjectVisionIO.ObjectObservation> allObservations = new ArrayList<>();
    private Set<CoralPose> coralPoses = new HashSet<>();

    public ObjectVision() {
        this.io = VisionConstants.getObjIO();
        this.inputs = new ObjectVisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new ObjectVisionIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < inputs.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("ObjectVision/" + io[i].getName(), inputs[i]);
            allObservations.addAll(Arrays.asList(inputs[i].observations));
        }
        allObservations.stream()
            .sorted(Comparator.comparingDouble(ObjectVisionIO.ObjectObservation::timestamp))
            .forEach(this::addCoralObservationToPose);

        coralObjects.setPoses(
            coralPoses.stream()
                .map(coral -> new Pose2d(coral.translation, new Rotation2d())) // zero rotation
                .collect(Collectors.toList()));
        allObservations.clear();
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
        Pose3d robotToCamera = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());

        // find object midpoint
        double tx = (observation.tx()[2] + observation.tx()[3]) / 2.0;
        double ty = (observation.ty()[2] + observation.ty()[3]) / 2.0;

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

        Pose2d fieldToCamera = fieldToRobot.transformBy(new Transform2d(robotToCamera.toPose2d().toMatrix()));
        Pose2d fieldToCoral =
            fieldToCamera
                .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-tx)))
                .transformBy(
                    new Transform2d(new Translation2d(cameraToObjectNorm, 0), Rotation2d.kZero));
        CoralPose coralPose = new CoralPose(fieldToCoral.getTranslation(), observation.timestamp());
        // delete coral once close enough to robot and deletes coral that has existed longer than 10 seconds
        coralPoses =
            coralPoses.stream()
                .filter(
                    (c) -> c.translation.getDistance(fieldToCoral.getTranslation()) > coralOverlap)
                .filter((c) -> now - c.timestamp <= coralMaxAge)
                .collect(Collectors.toSet());
        coralPoses.add(coralPose);
    }

    public void reset() {
        coralPoses.clear();
    }
}
