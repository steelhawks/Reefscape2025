package org.steelhawks.util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.Constants;

public class ButtonBoard {

    private final CommandGenericHID board;

    public ButtonBoard(int port) {
        board = new CommandGenericHID(port);
    }

    public Trigger getL1() {
        return board.button(Constants.OIConstants.L1_BUTTON_PORT);
    }

    public Trigger getL2() {
        return board.button(Constants.OIConstants.L2_BUTTON_PORT);
    }

    public Trigger getL3() {
        return board.button(Constants.OIConstants.L3_BUTTON_PORT);
    }

    public Trigger getL4() {
        return board.button(Constants.OIConstants.L4_BUTTON_PORT);
    }

    public Trigger getHome() {
        return board.button(Constants.OIConstants.HOME_BUTTON_PORT);
    }

    public Trigger getShoot() {
        return board.button(Constants.OIConstants.HOME_BUTTON_PORT);
    }
}
