package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.OneAutoToRuleThemAll;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

@Autonomous(name="Auto Far Vision Test (red)", group="testing")
public class AutoFarVisionTest extends OneAutoToRuleThemAll {
    public AutoFarVisionTest() {
        super(Team.RED);
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
                builder.prepareVision(),
                builder.cycleVision(true, ShootPathFlag.EARLY_LEAVE)
        );


    }
}
