package org.steelhawks.subsystems.climb.deep;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.steelhawks.subsystems.climb.ClimbConstants;

public class DeepClimbIO775Pro implements DeepClimbIO {

    private final TalonSRX mTopTalon;
    private final TalonSRX mBottomTalon;

    private final CANcoder mPivotEncoder;

    private final StatusSignal<Boolean> magnetFault;
    private final StatusSignal<Angle> canCoderPosition;
    private final StatusSignal<Angle> canCoderAbsolutePosition;
    private final StatusSignal<AngularVelocity> canCoderVelocity;

    public DeepClimbIO775Pro() {
        mTopTalon = new TalonSRX(ClimbConstants.DEEP_TOP_MOTOR_ID);
        mBottomTalon = new TalonSRX(ClimbConstants.DEEP_TOP_MOTOR_ID);
        mPivotEncoder = new CANcoder(ClimbConstants.DEEP_CANCODER_ID);

        magnetFault = mPivotEncoder.getFault_BadMagnet();
        canCoderPosition = mPivotEncoder.getPosition();
        canCoderAbsolutePosition = mPivotEncoder.getAbsolutePosition();
        canCoderVelocity = mPivotEncoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            magnetFault,
            canCoderPosition,
            canCoderVelocity);

        mTopTalon.setInverted(false);
        mBottomTalon.setInverted(true);

        mTopTalon.configVoltageCompSaturation(12.0);
        mBottomTalon.configVoltageCompSaturation(12.0);
        mTopTalon.enableVoltageCompensation(true);
        mBottomTalon.enableVoltageCompensation(true);
    }

    @Override
    public void updateInputs(DeepClimbIOInputs inputs) {
        inputs.connected = true;
        inputs.climbPositionRad = mTopTalon.getSelectedSensorPosition();
        inputs.climbVelocityRadPerSec = mTopTalon.getSelectedSensorVelocity();
        inputs.climbAppliedVolts = mTopTalon.getBusVoltage();
        inputs.climbCurrentAmps = mTopTalon.getStatorCurrent();

//        inputs.bottomConnected = true;
//        inputs.bottomClimbPositionRad = mBottomTalon.getSelectedSensorPosition();
//        inputs.bottomClimbVelocityRadPerSec = mBottomTalon.getSelectedSensorVelocity();
//        inputs.bottomClimbAppliedVolts = mBottomTalon.getBusVoltage();
//        inputs.bottomClimbCurrentAmps = mBottomTalon.getStatorCurrent();

        inputs.encoderConnected =
            BaseStatusSignal.refreshAll(
                magnetFault,
                canCoderPosition,
                canCoderVelocity).isOK();
        inputs.magnetGood = !magnetFault.getValue();
        inputs.encoderPositionRad = Units.rotationsToRadians(canCoderPosition.getValueAsDouble());
        inputs.encoderAbsolutePositionRad = Units.rotationsToRadians(canCoderAbsolutePosition.getValueAsDouble());
        inputs.encoderVelocityRadPerSec = Units.rotationsToRadians(canCoderVelocity.getValueAsDouble());
    }

    @Override
    public void runClimb(double volts) {
        mTopTalon.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
        mBottomTalon.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
    }

    @Override
    public void runClimbViaSpeed(double speed) {
        mTopTalon.set(TalonSRXControlMode.PercentOutput, speed);
        mBottomTalon.set(TalonSRXControlMode.PercentOutput, speed);
    }

    @Override
    public void stop() {
        mTopTalon.set(TalonSRXControlMode.PercentOutput, 0);
        mBottomTalon.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
