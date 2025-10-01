package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.init.RobotState.CLIMBING;
import static org.firstinspires.ftc.teamcode.robot.init.RobotState.RESTING;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Algebra;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorGamepad;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.init.StateTag;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;

@Config
public abstract class TerrorTeleOp extends LinearOpMode {

    public static double ROTATION_MULTIPLIER = 0.56;
    // If you use a manual override on the turret, it will take this long before it starts autoaiming again
    public static double TURRET_OVERRIDE_COOLDOWN = 2.0;

    private RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    private boolean isFieldCentric = false;
    private Pose2d goalPos;


    public void setFieldCentric(boolean fieldCentric) {
        this.isFieldCentric = fieldCentric;
    }
    public void setGoalPos(Pose2d goalPos) {this.goalPos = goalPos;}

    public void runOpMode(){
        if (isFieldCentric) {
            hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.PINPOINT);
        } else {
            hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        }
        robot.init(hardware, telemetry);

        waitForStart();

        TerrorGamepad gamepad1ex = new TerrorGamepad();
        TerrorGamepad gamepad2ex = new TerrorGamepad();

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime turretOverrideTimer = new ElapsedTime();

        robot.setState(RESTING);

        while (opModeIsActive()){
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            //update gamepads
            gamepad1ex.update(gamepad1);
            gamepad2ex.update(gamepad2);


            //driving, I just copied from last years teleop
            double deadzone_amt = 0;

            double left_x = gamepad1ex.left_stick_x();
            double left_y = -gamepad1ex.left_stick_y();
            double right_x = gamepad1ex.right_stick_x();

            left_x = Math.signum(left_x) * Algebra.mapRange(Math.abs(left_x), deadzone_amt, 1.0, 0.0, 1.0);
            left_y = Math.signum(left_y) * Algebra.mapRange(Math.abs(left_y), deadzone_amt, 1.0, 0.0, 1.0);
            right_x = Math.signum(right_x) * Algebra.mapRange(Math.abs(right_x), deadzone_amt, 1.0, 0.0, 1.0);

            Coordinate direction = new Coordinate(slr(left_x), slr(left_y));

            double rotation = slr(right_x)*ROTATION_MULTIPLIER;
            if (isFieldCentric) {
                robot.localizer.readIMU();
                double robotAngle = robot.localizer.getHeading();
                if (Double.isNaN(robotAngle)) {
                    // don't drive if NaN
                    direction = new Coordinate(0, 0);
                } else {
                    direction.rotate(-robotAngle);
                }
            }

        }
        //deadzone in 0.2, unless you are already adjusting the turret
        double deadzoneTurret = 0.2;
        if (turretOverrideTimer.seconds() < TURRET_OVERRIDE_COOLDOWN){
            deadzoneTurret = 0.0;
        }

        double right_y = gamepad1ex.right_stick_y();
        double pitch_angle = Algebra.mapRange(right_y, -1, 1.0, ShooterSubsystem.hoodAngleMin, ShooterSubsystem.hoodAngleMax);
        if (Math.abs(right_y) < deadzoneTurret){
            turretOverrideTimer.reset();
            robot.shooter.setPitch(pitch_angle);
        }

        if (robot.getState().checkTag(StateTag.FLYWHEEL_ON) && turretOverrideTimer.seconds() > 2.0){
            robot.shooter.doAutoShoot(robot.localizer.getPosition(), this.goalPos);
        }



        if (gamepad1ex.dpad_up(TerrorGamepad.State.RISING)){
            robot.setState(CLIMBING);
        }






    }
    public double slr(double joystick_value) {
        return Math.pow(joystick_value, 5);
    }



}
