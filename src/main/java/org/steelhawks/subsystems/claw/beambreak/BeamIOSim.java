package org.steelhawks.subsystems.claw.beambreak;

import org.steelhawks.subsystems.claw.ClawIOSim;

public class BeamIOSim implements BeamIO {
    @Override
    public void updateInputs(BeamIOInputs inputs) {
        inputs.connected = true;
        inputs.distance = ClawIOSim.mIntakeSim.getGamePiecesAmount() != 1 ? 0.001 : 10.0;
        inputs.broken = ClawIOSim.mIntakeSim.getGamePiecesAmount() == 1;
    }
}
