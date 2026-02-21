package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class LeaveAutoNear extends OneAutoToRuleThemAll {
    protected LeaveAutoNear(Team team) {
        super(team);
    }

    @Override
    protected StartConfig getStartConfig() {
        return StartConfig.NEAR;
    }

    @Override
    protected boolean wantsAutoSort() {
        return false;
    }

    @Override
    protected Command createAutoCommand(AutoBuilder builder) {
        return builder.shootPreload(ShootPathFlag.LAST);
    }
}
