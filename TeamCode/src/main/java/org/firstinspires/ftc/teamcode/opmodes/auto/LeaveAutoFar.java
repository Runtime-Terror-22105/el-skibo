package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuildState;
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
    public StartConfig getStartConfig() {
        return StartConfig.NEAR;
    }

    @Override
    public boolean wantsAutoSort() {
        return false;
    }

    @Override
    protected Command createAutoCommand(AutoBuildState state) {
        state.lastPath = PathUtil.createLinePath(robot, state.startPoseBlue, PARK_END, state.mirror, false, false);
        return new ParallelCommandGroup(
                new FollowPathCommand(robot.follower, state.lastPath),
                new GoToRestingStateCommand(robot)
        );
    }
}
