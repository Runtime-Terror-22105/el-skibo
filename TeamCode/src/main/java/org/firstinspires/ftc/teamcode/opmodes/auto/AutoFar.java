package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuildState;
import org.firstinspires.ftc.teamcode.robot.auto.FarAutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.KillTimerCommand;
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
    public boolean wantsBallCv() {
        return true;
    }

    @Override
    protected Command createAutoCommand(AutoBuildState state) {
        state.prepareShootTimeBeforeReverseIntake = 600;
        state.shootBrakingStrength = 0.75;
        return new SequentialCommandGroup(
//                new InstantCommand(() -> robot.spindexer.setTolerance(10)),
                FarAutoBuilder.shootPreloadFar(state),
                // Do not reverse intake on first since they're guaranteed
                FarAutoBuilder.cycleWall(state, false, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.intakeSpike3Far(state),
                FarAutoBuilder.shootSpike3Far(state, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.cycleWall(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.cycleTunnel(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.cycleWall(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.cycleWall(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.cycleTunnel(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.cycleWall(state, true, ShootPathFlag.LAST, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE)
        ).alongWith(new KillTimerCommand(robot));
    }
}