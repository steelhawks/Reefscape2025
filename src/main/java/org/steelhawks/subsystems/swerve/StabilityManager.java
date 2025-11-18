package org.steelhawks.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.util.LoggedTunableNumber;

public class StabilityManager {

    private static final StabilityManager INSTANCE = new StabilityManager();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Stability/kP", 0.01);
    private static final LoggedTunableNumber tippingLimitDeg = new LoggedTunableNumber("Stability/TippingLimitDeg", 3.0);
    private static final LoggedTunableNumber maxCorrectionSpeedMPS = new LoggedTunableNumber("Stability/MaxCorrectionSpeedMPS", 1.5);

    private Rotation2d pitch;
    private Rotation2d roll;

    public static StabilityManager getInstance() {
        return INSTANCE;
    }

    @AutoLogOutput(key = "Stability/IsStable")
    public boolean stabilityCheck(Rotation2d pitch, Rotation2d roll) {
        this.pitch = pitch;
        this.roll = roll;
        Logger.recordOutput("Stability/PitchDeg", pitch.getDegrees());
        Logger.recordOutput("Stability/RollDeg", roll.getDegrees());
        return !(Math.abs(pitch.getDegrees()) > tippingLimitDeg.getAsDouble() || Math.abs(roll.getDegrees()) > tippingLimitDeg.getAsDouble());
    }

    public ChassisSpeeds correction() {
        Rotation2d tiltVec = Rotation2d.fromRadians(Math.atan2(-roll.getRadians(), -pitch.getRadians()));
        double yawDir = tiltVec.getDegrees();

        double inclinationMagnitude = Math.hypot(pitch.getRadians(), roll.getRadians());
        double output = kP.getAsDouble() * -inclinationMagnitude;
        Logger.recordOutput("Stability/OutputRaw", output);
        output = MathUtil.clamp(output, -maxCorrectionSpeedMPS.getAsDouble(), maxCorrectionSpeedMPS.getAsDouble());
        Translation2d correction =
            new Translation2d(0, 1).rotateBy(tiltVec).times(output);
        Logger.recordOutput("Stability/CorrectionVector", correction);

        Logger.recordOutput("Stability/TiltDirectionDeg", yawDir);
        Logger.recordOutput("Stability/OutputClamped", output);
        return new ChassisSpeeds(correction.getX(), -correction.getY(), 0);
    }
}
