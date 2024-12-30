package org.firstinspires.ftc.teamcode.test;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name="wristTest",  group="Test")
public class wristtest extends OpMode {

    //pid
    ServoImplEx s1;
    ServoImplEx s2;

    public static double rw1 = .5;
    public static double rw2 =.36;

    public static double lw1 = .5;
    public static double lw2 =.36;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = (ServoImplEx) hardwareMap.get(Servo.class, "rw");
        s1.setPwmRange(new PwmControl.PwmRange(505, 2495));
        s2 = (ServoImplEx) hardwareMap.get(Servo.class, "lw");
        s2.setPwmRange(new PwmControl.PwmRange(505, 2495));



    }

    @Override
    public void loop() {

        //winch

        if (gamepad1.dpad_right) {
            s1.setPosition(rw1);
        }
        if (gamepad1.dpad_left) {
            s1.setPosition(rw2);
        }

        if (gamepad1.b) {
            s1.setPosition(lw1);
        }
        if (gamepad1.x) {
            s1.setPosition(lw2);
        }



        telemetry.addData("Run time", getRuntime());
        telemetry.addData("1", "org/firstinspires/ftc/teamcode/test");
        telemetry.update();
    }
}