package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class Auto18 extends OneAutoToRuleThemAll {
    protected Auto18(Team team) {
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
                builder.cycleSpike(2),
                builder.cycleGate(),
                builder.cycleGate(),
                builder.cycleSpike(3),
                builder.cycleSpike(1, ShootPathFlag.LAST)
        );
    }
}