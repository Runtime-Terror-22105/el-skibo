package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.KillTimerCommand;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class AutoSpamNoPushGate extends OneAutoToRuleThemAll {
    protected AutoSpamNoPushGate(Team team) {
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
    protected Command createAutoCommand(AutoBuilder builder) {
        builder.shootBrakingStrength = 0.7;
        return new SequentialCommandGroup(
                builder.shootPreload(ShootPathFlag.EARLY_LEAVE),
                builder.cycleSpike(1, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                builder.cycleSpike(2, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                builder.cycleGate(true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                builder.cycleGate(true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.LAST)
                //builder.cycleGate(true) //ShootPathFlag.LAST, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE)
        ).alongWith(new KillTimerCommand(robot));
    }
}