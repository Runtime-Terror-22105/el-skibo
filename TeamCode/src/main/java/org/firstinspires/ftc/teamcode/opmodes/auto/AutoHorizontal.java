package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class AutoHorizontal extends OneAutoToRuleThemAll {
    protected AutoHorizontal(Team team) {
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
        return new SequentialCommandGroup(
                builder.shootPreload(),
                builder.intakeSpikeHorizontal(1), builder.shootSpike(1),
                builder.intakeSpikeHorizontal(2), builder.shootSpike(2),
                builder.intakeSpikeHorizontal(3), builder.shootSpike(3, true)
        );
    }
}