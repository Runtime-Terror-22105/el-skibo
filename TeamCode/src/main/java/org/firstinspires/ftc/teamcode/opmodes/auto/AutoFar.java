package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class AutoFar extends OneAutoToRuleThemAll {
    protected AutoFar(Team team) {
        super(team);
    }

    @Override
    protected StartConfig getStartConfig() {
        return StartConfig.FAR;
    }

    @Override
    protected boolean wantsAutoSort() {
        return false;
    }

    @Override
    protected Command createAutoCommand(AutoBuilder builder) {
        return new SequentialCommandGroup(
                builder.shootPreloadFar(),
                // Do not intake on first since they're guaranteed
                builder.cycleWall(false),
                builder.intakeSpike3Far(), builder.shootSpike3Far(),
                builder.cycleTunnel(true),
                builder.cycleWall(true),
                builder.cycleTunnel(true),
                builder.cycleWall(true)
        );
    }
}