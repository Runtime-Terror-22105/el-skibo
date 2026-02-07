package org.firstinspires.ftc.teamcode.robot.command.shooter;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

// Comment copied from definition of autoShootPoseOverride in ShooterSubsystem:
//
// """
// If this is set, the robot will use this pose instead of the follower pose for auto-shoot
// calculations. If null, the follower pose is used.
//
// This is useful during auto to avoid dynamically updating the shooter while the robot moves.
// """
public class SetShooterPoseOverrideCommand extends InstantCommand {
    public SetShooterPoseOverrideCommand(ShooterSubsystem shooter, Pose robotPose) {
//        super(() -> shooter.autoShootPoseOverride = robotPose);
    }
}
