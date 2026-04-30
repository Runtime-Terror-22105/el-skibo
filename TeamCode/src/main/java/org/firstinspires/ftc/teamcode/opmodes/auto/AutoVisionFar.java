package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuildState;
import org.firstinspires.ftc.teamcode.robot.auto.FarAutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.KillTimerCommand;
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
    public boolean wantsCamera() {
        return true;
    }

    @Override
    protected Command createAutoCommand(AutoBuildState state) {
        state.prepareShootTimeBeforeReverseIntake = 0;
        state.shootBrakingStrength = 0.75;
        return new SequentialCommandGroup(
                FarAutoBuilder.shootPreload(state, ShootPathFlag.EARLY_LEAVE),

                // Do not reverse intake on first since they're guaranteed
                FarAutoBuilder.intakeSpike3(state),
                FarAutoBuilder.shootSpike3(state),

                FarAutoBuilder.cycleWall(state, false, ShootPathFlag.EARLY_SHOOT),

                FarAutoBuilder.prepareVision(state),
                FarAutoBuilder.cycleVision(state, true),
                FarAutoBuilder.cycleVision(state, true),
                FarAutoBuilder.cycleVision(state, true),
                FarAutoBuilder.cycleVision(state, true),
                FarAutoBuilder.cycleVision(state, true)
        ).alongWith(new KillTimerCommand(robot));
    }
}