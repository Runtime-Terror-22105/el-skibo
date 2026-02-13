package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.PathUtil;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.util.StartConfig;

@Config
public abstract class LeaveAutoFar extends OneAutoToRuleThemAll {
    public static Pose2d PARK_END = new Pose2d(30, 0, 1D/2D*Math.PI);

    protected LeaveAutoFar(Team team) {
        super(team);
    }

    @Override
    protected StartConfig getStartConfig() {
        return StartConfig.NEAR;
    }

    @Override
    protected boolean wantsAutoSort() {
        return false;
    }

    @Override
    protected Command createAutoCommand(AutoBuilder builder) {
        return new ParallelCommandGroup(
                new FollowPathCommand(
                        robot.follower,
                        builder.customPath(
                            PathUtil.createLinePath(robot, builder.startPoseBlue, PARK_END, builder.mirror, false, false)
                        )
                ),
                new GoToRestingStateCommand(robot)
        );
    }
}
