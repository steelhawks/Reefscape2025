package org.steelhawks.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants.AutonConstants;

public class SwerveDriveController {

    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
    private boolean firstRun = true;

    public SwerveDriveController(
        PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI); // see if changing to zero to two pi helps
    }

    public SwerveDriveController withLinearTolerance(double xyTolerance) {
        xController.setTolerance(xyTolerance);
        yController.setTolerance(xyTolerance);
        return this;
    }

    public SwerveDriveController withRotationalTolerance(double thetaTolerance) {
        thetaController.setTolerance(thetaTolerance);
        return this;
    }

    public ChassisSpeeds getOutput(Pose2d measurement, Pose2d setpoint) {
        if (firstRun) {
            thetaController.reset(measurement.getRotation().getRadians());
            firstRun = false;
        }

        double xFF = AutonConstants.MAX_VELOCITY_METERS_PER_SECOND * Math.cos(measurement.getRotation().getRadians());
        double yFF = AutonConstants.MAX_VELOCITY_METERS_PER_SECOND * Math.sin(measurement.getRotation().getRadians());
        Logger.recordOutput("SwerveDriveController/FeedforwardX", xFF);
        Logger.recordOutput("SwerveDriveController/FeedforwardY", yFF);

        double xOutput = xController.calculate(measurement.getX(), setpoint.getX());
        double yOutput = yController.calculate(measurement.getY(), setpoint.getY());
        double thetaOutput = thetaController.calculate(measurement.getRotation().getRadians(), setpoint.getRotation().getRadians());
        Logger.recordOutput("SwerveDriveController/OutputX", xOutput);
        Logger.recordOutput("SwerveDriveController/OutputY", yOutput);
        Logger.recordOutput("SwerveDriveController/OutputTheta", thetaOutput);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput + xFF,
            yOutput + yFF,
            thetaOutput,
            measurement.getRotation()
                .plus(new Rotation2d(AllianceFlip.shouldFlip() ? Math.PI : 0)));
    }

    public ChassisSpeeds getError() {
        return new ChassisSpeeds(
            xController.getErrorDerivative(),
            yController.getErrorDerivative(),
            thetaController.getVelocityError());
    }
}
