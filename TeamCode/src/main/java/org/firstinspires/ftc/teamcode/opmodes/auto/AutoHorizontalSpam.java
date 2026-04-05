package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuildState;
import org.firstinspires.ftc.teamcode.robot.auto.KillTimerCommand;
import org.firstinspires.ftc.teamcode.robot.auto.NearAutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class AutoHorizontalSpam extends OneAutoToRuleThemAll {
    protected AutoHorizontalSpam(Team team) {
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
        robot.shooter.sotmOverride = false;
        return new SequentialCommandGroup(
                NearAutoBuilder.shootPreload(state, ShootPathFlag.NEXT_HORIZ, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.EARLY_SHOOT),
                NearAutoBuilder.intakeSpikeHorizontal(state, 1), NearAutoBuilder.shootSpike(state, 1, ShootPathFlag.NEXT_HORIZ, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.EARLY_SHOOT),
                NearAutoBuilder.intakeSpikeHorizontal(state, 2), NearAutoBuilder.shootSpike(state, 2, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.EARLY_SHOOT),
                NearAutoBuilder.cycleGate(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.EARLY_SHOOT),
                NearAutoBuilder.cycleGate(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.EARLY_SHOOT),
                NearAutoBuilder.cycleGate(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.EARLY_SHOOT),
                NearAutoBuilder.cycleGate(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.LAST)
        ).alongWith(new KillTimerCommand(robot));
    }
}