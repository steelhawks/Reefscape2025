package org.steelhawks.subsystems.claw;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import org.steelhawks.util.SparkUtil;

public class ClawIOSparkFlex implements ClawIO {

    private static final int INTAKE_MOTOR_ID = 1;
    private static final int CURRENT_LIMIT = 80;

    private final SparkFlex mIntakeMotor;
    private final RelativeEncoder mIntakeEncoder;
    private final SparkBaseConfig mIntakeConfig;
    private final Debouncer mConnectedDebounce;

    public ClawIOSparkFlex() {
        mIntakeMotor = new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless);
        mIntakeEncoder = mIntakeMotor.getEncoder();
        mConnectedDebounce = new Debouncer(0.5);

        mIntakeConfig =
            new SparkFlexConfig()
                .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(CURRENT_LIMIT, 50)
            .voltageCompensation(12.0)
            .inverted(true);
        mIntakeConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
        mIntakeConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        SparkUtil.tryUntilOk(
            mIntakeMotor,
            5,
            () ->
                mIntakeMotor.configure(
                    mIntakeConfig,
                    SparkBase.ResetMode.kResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters));
        SparkUtil.tryUntilOk(mIntakeMotor, 5, () -> mIntakeEncoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(ClawIntakeIOInputs inputs) {
        inputs.connected = mConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);
        inputs.positionRad = Units.rotationsToRadians(mIntakeEncoder.getPosition());
        inputs.velocityRadPerSec = Units.rotationsToRadians(mIntakeEncoder.getVelocity() / 60.0);
        inputs.appliedVolts = mIntakeMotor.getBusVoltage();
        inputs.currentAmps = mIntakeMotor.getOutputCurrent();
        inputs.tempCelsius = mIntakeMotor.getMotorTemperature();
    }

    @Override
    public void runIntake(double percentageOutput) {
        mIntakeMotor.set(percentageOutput);
    }

    @Override
    public void stop() {
        mIntakeMotor.stopMotor();
    }
}
