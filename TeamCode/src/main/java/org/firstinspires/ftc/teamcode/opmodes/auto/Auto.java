package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class Auto extends OneAutoToRuleThemAll {
    protected Auto(Team team) {
        super(team);
    }

    @Override
    protected StartConfig getStartConfig() {
        return StartConfig.NEAR;
    }

    @Override
    protected boolean wantsAutoSort() {
        return true;
    }

    @Override
    protected Command createAutoCommand(AutoBuilder builder) {
        return new SequentialCommandGroup(
                builder.shootPreload(),
                builder.cycleSpike(1),
                builder.cycleSpike(2),
                builder.cycleSpike(3, true)
        );
    }
}