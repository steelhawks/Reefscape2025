package org.steelhawks.subsystems.intake.schlong;

import org.steelhawks.Constants;
import org.steelhawks.subsystems.intake.IntakeConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class SchlongIOTalonFX implements SchlongIO {
    private final IntakeConstants constants;

    private final TalonFX mSpinMotor;
    private final TalonFX mPivotMotor;

    private final DigitalInput mLimitSwitch;

    private final StatusSignal<Angle> spinPosition;
    private final StatusSignal<AngularVelocity> spinVelocity;
    private final StatusSignal<Voltage> spinVoltage;
    private final StatusSignal<Current> spinCurrent;
    private final StatusSignal<Temperature> spinTemp;

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotCurrent;
    private final StatusSignal<Temperature> pivotTemp;

    public SchlongIOTalonFX() {
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = IntakeConstants.ALPHA;
            case HAWKRIDER -> constants = IntakeConstants.HAWKRIDER;
            default -> constants = IntakeConstants.OMEGA;
        }

        mSpinMotor = new TalonFX(constants.SCHLONG_SPIN_MOTOR_ID);
        mPivotMotor = new TalonFX(constants.SCHLONG_PIVOT_MOTOR_ID);
        mLimitSwitch = new DigitalInput(constants.SCHLONG_LIMIT_SWITCH_ID);

        var spinConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(constants.SCHLONG_SPIN_GEAR_RATIO))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

        
        var pivotConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(constants.SCHLONG_PIVOT_GEAR_RATIO))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

        mSpinMotor.getConfigurator().apply(spinConfig);
        mPivotMotor.getConfigurator().apply(pivotConfig);

        zeroEncoders();

        spinPosition = mSpinMotor.getPosition();
        spinVelocity = mSpinMotor.getVelocity();
        spinVoltage = mSpinMotor.getSupplyVoltage();
        spinCurrent = mSpinMotor.getStatorCurrent();
        spinTemp = mSpinMotor.getDeviceTemp();

        pivotPosition = mPivotMotor.getPosition();
        pivotVelocity = mPivotMotor.getVelocity();
        pivotVoltage = mPivotMotor.getSupplyVoltage();
        pivotCurrent = mPivotMotor.getStatorCurrent();
        pivotTemp = mPivotMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            spinPosition,
            spinVelocity,
            spinVoltage,
            spinCurrent,
            spinTemp,

            pivotPosition,
            pivotVelocity,
            pivotVoltage,
            pivotCurrent,
            pivotTemp);

        ParentDevice.optimizeBusUtilizationForAll(mSpinMotor, mPivotMotor);
    }

    @Override
    public void updateInputs(SchlongIOInputs inputs) {
        double spinPos = spinPosition.getValueAsDouble();
        double pivotPos = pivotPosition.getValueAsDouble();

        double spinVelo = spinVelocity.getValueAsDouble();
        double pivotVelo = pivotVelocity.getValueAsDouble();

        inputs.spinConnected =
            BaseStatusSignal.refreshAll(
                spinPosition,
                spinVelocity,
                spinVoltage,
                spinCurrent,
                spinTemp).isOK();
        inputs.spinPositionRad = Units.rotationsToRadians(spinPos);
        inputs.spinVelocityRadPerSec = Units.rotationsToRadians(spinVelo);
        inputs.spinAppliedVolts = spinVoltage.getValueAsDouble();
        inputs.spinCurrentAmps = spinCurrent.getValueAsDouble();
        inputs.spinTempCelsius = spinTemp.getValueAsDouble();

        inputs.pivotConnected =
            BaseStatusSignal.refreshAll(
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotCurrent,
                pivotTemp).isOK();
        inputs.pivotPositionRad = Units.rotationsToRadians(pivotPos);
        inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelo);
        inputs.pivotAppliedVolts = pivotVoltage.getValueAsDouble();
        inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
        inputs.pivotTempCelsius = pivotTemp.getValueAsDouble();

        inputs.limitSwitchConnected = mLimitSwitch.getChannel() == constants.SCHLONG_LIMIT_SWITCH_ID;
        inputs.limitSwitchPressed = !mLimitSwitch.get();
    }

    @Override
    public void runPivotWithSpeed(double speed) {
        mPivotMotor.set(speed);
    }

    @Override
    public void runPivotWithVoltage(double volts) {
        mPivotMotor.set(volts);
    }

    @Override
    public void stopPivot() {
        mPivotMotor.stopMotor();
    }

    @Override
    public void runSpinWithSpeed(double speed) {
        mSpinMotor.set(speed);
    }

    @Override
    public void runSpinWithVoltage(double volts) {
        mSpinMotor.set(volts);
    }

    @Override
    public void stopSpin() {
        mSpinMotor.stopMotor();
    }


    @Override
    public void zeroEncoders() {
        mPivotMotor.setPosition(Units.radiansToRotations(- Math.PI / 2));
    }
}
