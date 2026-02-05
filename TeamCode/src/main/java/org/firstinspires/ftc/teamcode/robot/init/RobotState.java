package org.firstinspires.ftc.teamcode.robot.init;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.CLIMB;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.FLYWHEEL_OFF;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.FLYWHEEL_ON;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.FUNNEL_READY;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.INTAKE_FORWARD;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.INTAKE_REVERSE;
import static org.firstinspires.ftc.teamcode.robot.init.StateTag.INTAKE_UP;

import java.util.Arrays;
import java.util.List;

public enum RobotState {
    RESTING(new StateTag[] {INTAKE_UP, INTAKE_OFF, FLYWHEEL_OFF, FUNNEL_READY}),
    INTAKING(new StateTag[] {INTAKE_DOWN, INTAKE_FORWARD, FLYWHEEL_ON, FUNNEL_READY}),
    READY_TO_SHOOT(new StateTag[] {INTAKE_DOWN, FLYWHEEL_ON, FUNNEL_READY}), //when we have 3 balls
    SHOOTING(new StateTag[] {INTAKE_DOWN, INTAKE_REVERSE, FLYWHEEL_ON}),
    HANGING_90(new StateTag[] {INTAKE_UP, INTAKE_OFF, FLYWHEEL_OFF, CLIMB}),
    HANGING_FINAL(new StateTag[] {INTAKE_UP, INTAKE_OFF, FLYWHEEL_OFF, CLIMB}),

    TRANSFER(new StateTag[] {}),
    NOT_READY(new StateTag[]{});

    private final StateTag[] tags;
    RobotState(StateTag[] tags){ this.tags = tags;}

    public StateTag[] getTags() {
        return tags;
    }
    public boolean checkTag(StateTag tag){
        List<StateTag> list = Arrays.asList(tag);
        return list.contains(tag);
    }
}

