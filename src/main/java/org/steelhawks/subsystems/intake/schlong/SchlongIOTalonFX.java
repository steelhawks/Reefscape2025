package org.steelhawks.subsystems.intake.schlong;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.steelhawks.Constants;
import org.steelhawks.Constants.RobotType;
import org.steelhawks.subsystems.intake.IntakeConstants;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
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
    private final CANcoder mPivotEncoder;

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

    private StatusSignal<Boolean> magnetFault = null;
    private StatusSignal<Angle> canCoderPosition = null;
    private StatusSignal<Angle> canCoderAbsolutePosition = null;
    private StatusSignal<AngularVelocity> canCoderVelocity = null;


    public SchlongIOTalonFX() {
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = IntakeConstants.ALPHA;
            case HAWKRIDER -> constants = IntakeConstants.HAWKRIDER;
            default -> constants = IntakeConstants.OMEGA;
        }

        mSpinMotor = new TalonFX(constants.SCHLONG_SPIN_MOTOR_ID, Constants.getCANBus());
        mPivotMotor = new TalonFX(constants.SCHLONG_PIVOT_MOTOR_ID, Constants.getCANBus());
        if (constants.SCHLONG_LIMIT_SWITCH_ID != -1) {
            mLimitSwitch = new DigitalInput(constants.SCHLONG_LIMIT_SWITCH_ID);
        } else {
            mLimitSwitch = null;
        }
        if (constants.SCHLONG_CANCODER_ID != -1) {
            mPivotEncoder = new CANcoder(constants.SCHLONG_CANCODER_ID, Constants.getCANBus());

            magnetFault = mPivotEncoder.getFault_BadMagnet();
            canCoderPosition = mPivotEncoder.getPosition();
            canCoderAbsolutePosition = mPivotEncoder.getAbsolutePosition();
            canCoderVelocity = mPivotEncoder.getVelocity();

            var encoderConfig =
                new CANcoderConfiguration().MagnetSensor
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
            mPivotEncoder.getConfigurator().apply(encoderConfig);

            BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                magnetFault,
                canCoderPosition,
                canCoderAbsolutePosition,
                canCoderVelocity);
            mPivotEncoder.optimizeBusUtilization();
        } else {
            mPivotEncoder = null;
        }

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
        inputs.spinConnected =
            BaseStatusSignal.refreshAll(
                spinPosition,
                spinVelocity,
                spinVoltage,
                spinCurrent,
                spinTemp).isOK();
        inputs.spinPositionRad = Units.rotationsToRadians(spinPosition.getValueAsDouble());
        inputs.spinVelocityRadPerSec = Units.rotationsToRadians(spinVelocity.getValueAsDouble());
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
        inputs.pivotPositionRad = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
        inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
        inputs.pivotAppliedVolts = pivotVoltage.getValueAsDouble();
        inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
        inputs.pivotTempCelsius = pivotTemp.getValueAsDouble();

        if (mPivotEncoder != null) {
            inputs.encoderConnected =
                BaseStatusSignal.refreshAll(
                    magnetFault,
                    canCoderPosition,
                    canCoderAbsolutePosition,
                    canCoderVelocity).isOK();
            inputs.magnetGood = !magnetFault.getValue();
            inputs.encoderPositionRad = Units.rotationsToRadians(canCoderPosition.getValueAsDouble());
            inputs.encoderAbsolutePositionRad = Units.rotationsToRadians(canCoderAbsolutePosition.getValueAsDouble())   ;
            inputs.encoderVelocityRadPerSec = canCoderVelocity.getValueAsDouble();
        }

        if (mLimitSwitch != null) {
            inputs.limitSwitchConnected = mLimitSwitch.getChannel() == constants.SCHLONG_LIMIT_SWITCH_ID;
            inputs.limitSwitchPressed = !mLimitSwitch.get();    
        }
    }

    @Override
    public void runPivotWithSpeed(double speed) {
        mPivotMotor.set(speed);
    }

    @Override
    public void runPivotWithVoltage(String runningHere, double volts) {
        System.out.println(runningHere);
        mPivotMotor.setVoltage(volts);
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
        if (Constants.getRobot() == RobotType.ALPHABOT)
            mPivotMotor.setPosition(Units.radiansToRotations(-Math.PI / 2));

        if (mPivotEncoder != null)
            mPivotEncoder.setPosition(0);
    }
}
