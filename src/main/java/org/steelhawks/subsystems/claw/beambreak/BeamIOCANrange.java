package org.steelhawks.subsystems.claw.beambreak;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.claw.ClawConstants;

public class BeamIOCANrange implements BeamIO {

    private final CANrange mBeamBreak;

    private final StatusSignal<Distance> distance;
    private final StatusSignal<Boolean> beamBroken;

    public BeamIOCANrange() {
        mBeamBreak = new CANrange(ClawConstants.CAN_RANGE_ID_OMEGA, "");
        var canRangeConfig =
            new CANrangeConfiguration()
                .withProximityParams(new ProximityParamsConfigs()
                    .withProximityThreshold(Claw.DIST_TO_HAVE_CORAL));
        mBeamBreak.getConfigurator().apply(canRangeConfig);
        distance = mBeamBreak.getDistance();
        beamBroken = mBeamBreak.getIsDetected();
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            distance,
            beamBroken);
        mBeamBreak.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(BeamIOInputs inputs) {
        inputs.connected =
            BaseStatusSignal.refreshAll(
                distance,
                beamBroken).isOK();
        inputs.distance = distance.getValueAsDouble();
        inputs.broken = beamBroken.getValue();
    }
}
