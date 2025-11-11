package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp(name = "Servo Test", group = "Testing")
@Config
public class ServoTest extends LinearOpMode {
    // targets
    public static double TURRET_YAW_LEFT_POS = 0.5;
    public static double TURRET_YAW_RIGHT_POS = 0.5;
    public static double SHOOTER_PITCH_POS = 0.5;
    public static double SPINDEXER_INTAKE_RAMP_1_POS = 0.5;
    public static double SPINDEXER_INTAKE_RAMP_2_POS = 0.5;
    public static double SPINDEXER_DIDDY_POS = 0.5;
    public static double SPINDEXER_TRANSFER_RAMP_POS = 0.5;
    public static double INTAKE_PITCH_1_POS = 0.5;
    public static double INTAKE_PITCH_2_POS = 0.5;
    public static double SPINDEXER_PTO_POS = 0.5;

    public static boolean doTurret = false;
    public static boolean doShooterPitch = false;
    public static boolean doSpindexerIntakeRamps = false;
    public static boolean doSpindexerDiddy = false;
    public static boolean doSpindexerTransfer = false;
    public static boolean doIntakePitch = false;
    public static boolean doSpindexerPto = false;

    private final RobotHardware hardware = new RobotHardware();

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        waitForStart();

        while (opModeIsActive()) {
            if (hardware.allHubs != null) {
                for (LynxModule hub : hardware.allHubs) {
                    hub.clearBulkCache();
                }
            }

            if (doTurret) {
                if (hardware.turretYawLeft != null) hardware.turretYawLeft.setPosition(TURRET_YAW_LEFT_POS);
                if (hardware.turretYawRight != null) hardware.turretYawRight.setPosition(TURRET_YAW_RIGHT_POS);
            }
            if (doShooterPitch && hardware.shooterPitch != null) {
                hardware.shooterPitch.setPosition(SHOOTER_PITCH_POS);
            }
            if (doSpindexerIntakeRamps) {
                if (hardware.spindexerIntakeRampServo1 != null) hardware.spindexerIntakeRampServo1.setPosition(SPINDEXER_INTAKE_RAMP_1_POS);
                if (hardware.spindexerIntakeRampServo2 != null) hardware.spindexerIntakeRampServo2.setPosition(SPINDEXER_INTAKE_RAMP_2_POS);
            }
            if (doSpindexerDiddy && hardware.spindexerDiddyServo != null) {
                hardware.spindexerDiddyServo.setPosition(SPINDEXER_DIDDY_POS);
            }
            if (doSpindexerTransfer && hardware.spindexerTransferRampServo != null) {
                hardware.spindexerTransferRampServo.setPosition(SPINDEXER_TRANSFER_RAMP_POS);
            }
            if (doIntakePitch) {
                if (hardware.intakePitch1 != null) hardware.intakePitch1.setPosition(INTAKE_PITCH_1_POS);
                if (hardware.intakePitch2 != null) hardware.intakePitch2.setPosition(INTAKE_PITCH_2_POS);
            }
            if (doSpindexerPto && hardware.spindexerPTO != null) {
                hardware.spindexerPTO.setPosition(SPINDEXER_PTO_POS);
            }

            hardware.write();
        }
    }
}