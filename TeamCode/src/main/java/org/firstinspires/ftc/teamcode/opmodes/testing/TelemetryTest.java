package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FastTelemetryImpl;

@TeleOp(name = "Telemetry performance test")
public class TelemetryTest extends LinearOpMode {

    // Make telemetry go nyyyyOOOOOOMMMMM
    Telemetry telemetry = new FastTelemetryImpl(this);

    public static void timed(Telemetry telemetry, TelemetryPacket packet, String name, Runnable fn) {
        telemetry.addData("<big><b><u>" + name + "</big></b></u>", "Start");
        ElapsedTime timer = new ElapsedTime();

        fn.run();

        double elapsedMs = timer.milliseconds();

        String color = elapsedMs < 5? "green" : elapsedMs < 10? "yellow" : elapsedMs < 20? "#FFA500" : "#FF3333";

        telemetry.addData("<b><u>" + name + "</b></u>", "Finish (<font color=\""+ color + "\"><b>%.1f ms</b></font>)", elapsedMs);
        telemetry.addLine();

        // Attempt to send to FTC Dashboard for graphing
        if (packet != null)
            packet.put(name, elapsedMs);
    }

    ElapsedTime totalLoopTimeTimer = new ElapsedTime();
    double lastLoopTime = 0;

    ElapsedTime telemetryUpdateTime = new ElapsedTime();

    int lastLoopTimeLongerThan20ms = 0;
    int lastLoopTimeLongerThan30ms = 0;
    int lastLoopTimeLongerThan40ms = 0;
    int lastLoopTimeLongerThan60ms = 0;
    int lastLoopTimeLongerThan100ms = 0;

    TelemetryPacket timingDataPacket = new TelemetryPacket();

    public void timed (String name, Runnable fn) {
        timed(telemetry, timingDataPacket, name, fn);
    }

    void printDummy() {
        telemetry.addData("bleh", "dummy thing");
        telemetry.addData("bleh", "dummy thing");
        telemetry.addData("bleh", "dummy thing");
        telemetry.addData("bleh", "dummy thing");
        telemetry.addData("bleh", "dummy thing");
        telemetry.addData("bleh", "dummy thing");
    }

    @Override
    public void runOpMode() {

        // Enable HTML mode for Telemetry
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        // Make telemetry.update() always update, for testing (comment/un-comment as needed)
        telemetry.setMsTransmissionInterval(1);

        waitForStart();

        totalLoopTimeTimer.reset();

        while (opModeIsActive()) {
            timed("Update all subsystems", () -> {
                timed("Update fast subsystem", () -> { printDummy(); sleep(2);   } );
                timed("Update eh subsystem",   () -> { printDummy(); sleep(8);   } );
                timed("Update hmm subsystem",  () -> { printDummy(); sleep(15);  } );
//                timed("Update bleh subsystem", () -> { printDummy(); sleep(250); } );
            });

            lastLoopTime = totalLoopTimeTimer.milliseconds();
            totalLoopTimeTimer.reset();

            // increment number if too long loop
            if (lastLoopTime > 20)
                lastLoopTimeLongerThan20ms++;
            if (lastLoopTime > 30)
                lastLoopTimeLongerThan30ms++;
            if (lastLoopTime > 40)
                lastLoopTimeLongerThan40ms++;
            if (lastLoopTime > 60)
                lastLoopTimeLongerThan60ms++;
            if (lastLoopTime > 100)
                lastLoopTimeLongerThan100ms++;

            String color = lastLoopTime < 20? "green" : lastLoopTime < 40? "yellow" : lastLoopTime < 60? "#FFA500" : "#FF3333";

            telemetry.addData("<big><b><u>Total loop time</big></b></u>", "<font color=\""+ color + "\"><b>%.1f ms</b></font>", lastLoopTime);
            telemetry.addData("Loops longer than 20ms", lastLoopTimeLongerThan20ms);
            telemetry.addData("Loops longer than 30ms", lastLoopTimeLongerThan30ms);
            telemetry.addData("Loops longer than 40ms", lastLoopTimeLongerThan40ms);
            telemetry.addData("Loops longer than 60ms", lastLoopTimeLongerThan60ms);
            telemetry.addData("Loops longer than 100ms", lastLoopTimeLongerThan100ms);

            telemetryUpdateTime.reset();
            int updated = telemetry.update()? 1 : 0;
            timingDataPacket.put("Telemetry update time", telemetryUpdateTime.milliseconds());
            timingDataPacket.put("Did telemetry update?", updated);

            // Send to FTC Dashboard for graphing
            timingDataPacket.put("Total loop time", lastLoopTime);
            FtcDashboard.getInstance().sendTelemetryPacket(timingDataPacket);
        }

    }

}