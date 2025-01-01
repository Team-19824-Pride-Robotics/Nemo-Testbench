package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
@Disabled
@TeleOp(name="improveLoopTime")
public class improveLoopTime extends OpMode {

    //https://cookbook.dairy.foundation/improving_loop_times/improving_loop_times.html
    //https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html

    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }
    @Override
    public void loop() {
        for (LynxModule hub : allHubs){
            hub.clearBulkCache();
        }

        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
        telemetry.update();
    }
}
