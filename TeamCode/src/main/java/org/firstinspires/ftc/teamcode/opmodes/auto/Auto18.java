package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuildState;
import org.firstinspires.ftc.teamcode.robot.auto.KillTimerCommand;
import org.firstinspires.ftc.teamcode.robot.auto.NearAutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class Auto18 extends OneAutoToRuleThemAll {
    protected Auto18(Team team) {
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
    protected Command createAutoCommand(AutoBuildState state) {
        return new SequentialCommandGroup(
                NearAutoBuilder.shootPreload(state, ShootPathFlag.EARLY_LEAVE),
                NearAutoBuilder.cycleSpike(state, 2, ShootPathFlag.EARLY_LEAVE),
                NearAutoBuilder.cycleGate(state, true, ShootPathFlag.EARLY_LEAVE),
                NearAutoBuilder.cycleGate(state, true, ShootPathFlag.EARLY_LEAVE),
                NearAutoBuilder.cycleGate(state, true, ShootPathFlag.EARLY_LEAVE),
                NearAutoBuilder.cycleSpike(state, 1, ShootPathFlag.LAST)
        ).alongWith(new KillTimerCommand(robot));
    }
}