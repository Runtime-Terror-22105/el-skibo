package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class AutoFar extends OneAutoToRuleThemAll {
    protected AutoFar(Team team) {
        super(team);
    }

    @Override
    public StartConfig getStartConfig() {
        return StartConfig.FAR;
    }

    @Override
    public boolean wantsAutoSort() {
        return false;
    }

    @Override
    protected Command createAutoCommand(AutoBuilder builder) {
        return new SequentialCommandGroup(
                builder.shootPreloadFar(ShootPathFlag.EARLY_LEAVE),
                // Do not reverse intake on first since they're guaranteed
                builder.cycleWall(false, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                builder.intakeSpike3Far(),
                builder.shootSpike3Far(ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                builder.cycleTunnel(true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                builder.cycleWall(true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                builder.cycleTunnel(true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                builder.cycleWall(true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                builder.cycleTunnel(true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                builder.cycleWall(true, ShootPathFlag.LAST, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE)
        );
    }
}