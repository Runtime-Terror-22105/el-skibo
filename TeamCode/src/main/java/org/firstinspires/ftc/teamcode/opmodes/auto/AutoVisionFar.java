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
    public boolean wantsBallCv() {
        return true;
    }

    @Override
    protected Command createAutoCommand(AutoBuildState state) {
        state.prepareShootTimeBeforeReverseIntake = 0;
        state.shootBrakingStrength = 0.75;
        return new SequentialCommandGroup(
                FarAutoBuilder.shootPreload(state),

                // Do not reverse intake on first since they're guaranteed
                FarAutoBuilder.intakeSpike3(state),
                FarAutoBuilder.shootSpike3(state, ShootPathFlag.EARLY_LEAVE),

                FarAutoBuilder.cycleWall(state, false, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),

                FarAutoBuilder.prepareVision(state),
                FarAutoBuilder.cycleVision(state, true, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.cycleVision(state, true, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.cycleVision(state, true, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.cycleVision(state, true, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.cycleVision(state, true, ShootPathFlag.EARLY_LEAVE)
        ).alongWith(new KillTimerCommand(robot));
    }
}