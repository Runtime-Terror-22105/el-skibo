package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuildState;
import org.firstinspires.ftc.teamcode.robot.auto.KillTimerCommand;
import org.firstinspires.ftc.teamcode.robot.auto.NearAutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class AutoSpam extends OneAutoToRuleThemAll {
    protected AutoSpam(Team team) {
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
                NearAutoBuilder.shootPreload(state, ShootPathFlag.SOTM, ShootPathFlag.EARLY_LEAVE),
                NearAutoBuilder.cycleSpike(state, 1, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                NearAutoBuilder.intakeSpike(state, 2),
                NearAutoBuilder.pushGate(state),
                NearAutoBuilder.shootSpike(state, 2, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                NearAutoBuilder.cycleGate(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                NearAutoBuilder.cycleGate(state, true, ShootPathFlag.LAST, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE)
        ).alongWith(new KillTimerCommand(robot));
    }
}