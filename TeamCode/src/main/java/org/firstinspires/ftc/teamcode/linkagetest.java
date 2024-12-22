package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name="linkageTest")
public class linkagetest extends OpMode {

    //pid
    ServoImplEx s1;
    ServoImplEx s2;

    public static double rl1 = .2;
    public static double rl2 =.36;

    public static double ll1 = .2;
    public static double ll2 =.36;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = (ServoImplEx) hardwareMap.get(Servo.class, "rl");
        s1.setPwmRange(new PwmControl.PwmRange(505, 2495));
        s2 = (ServoImplEx) hardwareMap.get(Servo.class, "ll");
        s2.setPwmRange(new PwmControl.PwmRange(505, 2495));



    }

    @Override
    public void loop() {

        //winch

        if (gamepad1.dpad_right) {
            s1.setPosition(rl1);
        }
        if (gamepad1.dpad_left) {
            s1.setPosition(rl2);
        }

        if (gamepad1.b) {
            s1.setPosition(ll1);
        }
        if (gamepad1.x) {
            s1.setPosition(ll2);
        }



        telemetry.addData("Run time", getRuntime());
        telemetry.addData("1", "test");
        telemetry.update();
    }
}