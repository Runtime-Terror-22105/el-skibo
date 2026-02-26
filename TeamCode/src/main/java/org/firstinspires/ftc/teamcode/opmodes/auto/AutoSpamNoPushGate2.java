package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class AutoSpamNoPushGate2 extends OneAutoToRuleThemAll {
    protected AutoSpamNoPushGate2(Team team) {
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
                builder.shootPreload(ShootPathFlag.SOTM),
                builder.cycleSpike(1),
                builder.cycleSpike(2),
                builder.cycleGate(),
                builder.cycleGate(ShootPathFlag.LAST)
        );
    }
}