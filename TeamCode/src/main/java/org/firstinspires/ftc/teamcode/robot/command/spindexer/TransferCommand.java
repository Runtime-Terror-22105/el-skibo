package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

@Config
public class TransferCommand extends SequentialCommandGroup {
    public static double SHOOTER_POWER = 1.0;
    public static double SHOOTER_RPM = 3500;
    public static double PRE_YAW_ANGLE = 30.0;  // degrees
    public static int PRE_YAW_DELAY = 250;  // milliseconds
    public static int RAMP_DELAY = 500;  // milliseconds
    public static int TRANSFER_TIME = 2000;  // milliseconds
    public static double SPINDEX_ROTATIONS = -4.5;  // revolutions, negative bc clockwise

    private final Robot robot;

    public TransferCommand(Robot robot) {
        super(
                // Phase 1 and 2: ???
                new SetSpindexerWallDown(robot.spindexer, false),
                new SetSpindexerPoleActive(robot.spindexer, true),
                new InstantCommand(() -> robot.shooter.setVelocity(SHOOTER_RPM)),
                new WaitCommand(500), // todo: adjust this delay based on how long it takes for these two servos

                // Phase 3: rotate to pre-transfer yaw
                // TODO: angle needs to be relative to current position, NOT absolute
                new SetSpindexerYawCommand(robot.spindexer, Math.toRadians(PRE_YAW_ANGLE)),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(PRE_YAW_DELAY),

                // Phase 4: drop down ramp and start intake
                new SetSpindexerRampActive(robot.spindexer, true),
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.DEFAULT_SPEED),
//                new InstantCommand(() -> {
//                    robot.hardware.shooterLeft.setPower(SHOOTER_POWER);
//                    robot.hardware.shooterRight.setPower(SHOOTER_POWER);
//                }),
                new WaitCommand(RAMP_DELAY), // todo: adjust this delay based on how long it takes for ramp to drop

                // Phase 5: transfer balls
//                new InstantCommand(() -> {
//                    robot.spindexer.setPidEnabled(false);
//                    robot.spindexer.setSpindexerPower(SpindexerSubsystem.TRANSFER_POWER);
//                }),
                new ChangeSpindexerYawCommand(robot.spindexer, SPINDEX_ROTATIONS*2*Math.PI),
                new ParallelRaceGroup( // keep going for either 2 rotations or until all balls are gone
                    new WaitForSpindexerYawCommand(robot.spindexer)
//                    new WaitUntilCommand(() -> {
//                        char[] balls = robot.spindexer.getBallPositions();
//                        return balls[0] == 'N' && balls[1] == 'N' && balls[2] == 'N';
//                    })
                ),

                // reset spindexer, intake, shooter, and pole
                new ParallelCommandGroup(
                    new InstantCommand(() -> robot.spindexer.setYaw(robot.spindexer.getPosition())),
                    new SetIntakeSpeedCommand(robot.intake, 0),
                    new InstantCommand(() -> {
                        robot.hardware.shooterLeft.setPower(0);
                        robot.hardware.shooterRight.setPower(0);
                    }),
                    new SetSpindexerPoleActive(robot.spindexer, false),
                    new SetSpindexerRampActive(robot.spindexer, false)
                )
        );
        this.robot = robot;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robot.spindexer.setPidEnabled(true);
        robot.intake.setSpeed(0);
        robot.hardware.shooterLeft.setPower(0);
        robot.hardware.shooterRight.setPower(0);
    }

    //    public void phase1() {
//        spindexer.setWallActive();
//        spindexer.Oilup();
//    }

//    public void phase2(){
//        spindexer.sortBalls();
//    }
//
//    public void phase3(){
//        spindexer.enableRamp();
//    }
//
//    public void phase4(){
//        spindexer.setYaw(0.20);
//    }
//
//    public void phase5(){
//        spindexer.setSpindexerPower(spindexer.spindexTransferPower);
//        spindexer.sortBalls();
//    }
//
//    public void execute(){
//        Log.d("transfer", "command running");
//        SequentialCommandGroup group = new SequentialCommandGroup(
//                new InstantCommand(() -> phase1() ),
//                new InstantCommand(() -> phase2() ),
//                new InstantCommand(() -> phase3() ),
//                new InstantCommand(() -> phase4() ),
//                new InstantCommand(() -> phase5())
//        );
//        CommandScheduler.getInstance().schedule(group);
//
//
//
//    }


}
