package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

@Config
public abstract class AutoSorted12 extends OneAutoToRuleThemAll {
    public static long SHOOTING_DELAY = 1000;

    protected AutoSorted12(Team team) {
        super(team);
    }

    @Override
    public StartConfig getStartConfig() {
        return StartConfig.NEAR;
    }

    @Override
    public boolean wantsAutoSort() {
        return true;
    }

    @Override
    protected Command createAutoCommand(AutoBuilder builder) {
        return new SequentialCommandGroup(
                builder.shootPreload(),
                new WaitCommand(SHOOTING_DELAY),
                builder.cycleSpike(1),
                new WaitCommand(SHOOTING_DELAY),
                builder.cycleSpike(2),
                new WaitCommand(SHOOTING_DELAY),
                builder.cycleSpike(3, ShootPathFlag.LAST)
        );
    }
}