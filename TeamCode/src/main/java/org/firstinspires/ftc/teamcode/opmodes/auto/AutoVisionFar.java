package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class AutoVisionFar extends OneAutoToRuleThemAll {
    protected AutoVisionFar(Team team) {
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
                builder.cycleWall(false, ShootPathFlag.EARLY_LEAVE),
                builder.intakeSpike3Far(),
                builder.shootSpike3Far(ShootPathFlag.EARLY_LEAVE),
                builder.prepareVision(),
                builder.cycleVision(true, ShootPathFlag.EARLY_LEAVE),
                builder.cycleVision(true, ShootPathFlag.EARLY_LEAVE),
                builder.cycleVision(true, ShootPathFlag.EARLY_LEAVE),
                builder.cycleVision(true, ShootPathFlag.EARLY_LEAVE),
                builder.cycleVision(true, ShootPathFlag.EARLY_LEAVE),
                builder.cycleVision(true, ShootPathFlag.EARLY_LEAVE)


        );


    }
}