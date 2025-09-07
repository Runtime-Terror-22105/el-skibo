package org.firstinspires.ftc.teamcode.robot.init;

public enum RobotState {
    RESTING("r_t"),
//    RESTING_RESET_ENCODER("r_t_e"),
    AUTO_RESTING("r_a"),

    INTAKE("i_sub"),

    HIGH_BUCKET("b_high"),
    LOW_BUCKET("b_low"),
    SAMPLE_INTERMEDIARY_UP("b_i_up"),
    SAMPLE_INTERMEDIARY_DOWN("b_i_1"),

    // Endgame stuff
    HANG_BOXTUBE_READY("e_1"), // note that the hang hooks should be deployed already
    HANG_SLIGHT_RETRACTED("e_1_retracted"),
    HANG_HOOKS_DOWN_PTO_READY("e_2"),
    HANG_LEVEL_2("e_3");

    private final String stateId;

    RobotState(String stateId) {
        this.stateId = stateId;
    }

    public boolean isHanging() {
        return this.stateId.charAt(0) == 'e';
    }

    public boolean isDepositState() {
        return this.stateId.charAt(0) == 'b';
    }
}
