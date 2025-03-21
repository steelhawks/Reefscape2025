package org.steelhawks.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import java.util.function.Supplier;

public class FieldBoundingBox extends Trigger {

    public FieldBoundingBox(
        String name,
        double minX,
        double maxX,
        double minY,
        double maxY,
        Supplier<Pose2d> robotPose
    ) {
        super(() -> {
            double dynamicMinX = AllianceFlip.applyX(minX);
            double dynamicMaxX = AllianceFlip.applyX(maxX);
            double dynamicMinY = AllianceFlip.applyY(minY);
            double dynamicMaxY = AllianceFlip.applyY(maxY);

            return (robotPose.get().getX() >= dynamicMinX && robotPose.get().getX() <= dynamicMaxX)
                && (robotPose.get().getY() >= dynamicMinY && robotPose.get().getY() <= dynamicMaxY);
        });

        if (Constants.getRobot() != Constants.RobotType.SIMBOT)
            return;

        new VirtualSubsystem("FieldBoundingBox/" + name) {
            @Override
            public void periodic() {
                double dynamicMinX = AllianceFlip.applyX(minX);
                double dynamicMaxX = AllianceFlip.applyX(maxX);
                double dynamicMinY = AllianceFlip.applyY(minY);
                double dynamicMaxY = AllianceFlip.applyY(maxY);

                Logger.recordOutput("FieldBoundingBox/" + name + "/MinX", dynamicMinX);
                Logger.recordOutput("FieldBoundingBox/" + name + "/MaxX", dynamicMaxX);
                Logger.recordOutput("FieldBoundingBox/" + name + "/MinY", dynamicMinY);
                Logger.recordOutput("FieldBoundingBox/" + name + "/MaxY", dynamicMaxY);

                Logger.recordOutput("FieldBoundingBox/" + name + "/Box",
                    new Pose2d(dynamicMinX, dynamicMinY, new Rotation2d()),
                    new Pose2d(dynamicMaxX, dynamicMinY, new Rotation2d()),
                    new Pose2d(dynamicMinX, dynamicMaxY, new Rotation2d()),
                    new Pose2d(dynamicMaxX, dynamicMaxY, new Rotation2d()));
            }
        };
    }
}
