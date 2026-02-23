package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class AutoHorizontal extends OneAutoToRuleThemAll {
    protected AutoHorizontal(Team team) {
        super(team);
    }

    @Override
    public StartConfig getStartConfig() {
        return StartConfig.NEAR;
    }

    @Override
    public boolean wantsAutoSort() {
        return false;
    }

    @Override
    protected Command createAutoCommand(AutoBuilder builder) {
        return new SequentialCommandGroup(
                builder.shootPreload(ShootPathFlag.NEXT_HORIZ),
                builder.intakeSpikeHorizontal(1), builder.shootSpike(1, ShootPathFlag.NEXT_HORIZ),
                builder.intakeSpikeHorizontal(2), builder.shootSpike(2, ShootPathFlag.NEXT_HORIZ),
                builder.intakeSpikeHorizontal(3), builder.shootSpike(3, ShootPathFlag.LAST)
        );
    }
}