package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuildState;
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
                NearAutoBuilder.shootPreload(state),
                NearAutoBuilder.cycleSpike(state, 2),
                NearAutoBuilder.cycleGate(state, true),
                NearAutoBuilder.cycleGate(state, true),
                NearAutoBuilder.cycleGate(state, true),
                NearAutoBuilder.cycleSpike(state, 3),
                NearAutoBuilder.cycleSpike(state, 1, ShootPathFlag.LAST)
        );
    }
}