package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.OneAutoToRuleThemAll;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuildState;
import org.firstinspires.ftc.teamcode.robot.auto.FarAutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

@Autonomous(name="Auto Far Vision Test (red)", group="Testing")
public class AutoFarVisionTest extends OneAutoToRuleThemAll {
    public AutoFarVisionTest() {
        super(Team.RED);
    }

    // start the robot facing towards the wall
    @Override
    public StartConfig getStartConfig() {
        return StartConfig.FAR_SIDE;
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
        return new SequentialCommandGroup(
//                FarAutoBuilder.shootPreload(state, ShootPathFlag.EARLY_LEAVE),
                FarAutoBuilder.prepareVision(state),
                FarAutoBuilder.cycleVision(state, true, ShootPathFlag.EARLY_LEAVE)
        );


    }
}
