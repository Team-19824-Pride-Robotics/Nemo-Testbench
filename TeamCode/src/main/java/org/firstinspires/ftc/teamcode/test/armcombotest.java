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
@TeleOp(name="armcomboTest",  group="Test")
public class armcombotest extends OpMode {


    ServoImplEx s1;
    ServoImplEx s2;

    public static double rw1 = .46;
    public static double rw2 =.38;

    public static double lw1 = .5;
    public static double lw2 =.42;

    public static double rwpickup = .36;
    public static double lwpickup = .40;

    public static double rwscore = .46;
    public static double lwscore = .5;


    ServoImplEx s3;
    public static double clawopen = .82;//pid
    public static double clawclose = .935;//pid

    ServoImplEx s0;

    public static double armpickup = .30;
    public static double armscore = .7;
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s0 = (ServoImplEx) hardwareMap.get(Servo.class, "arm");
        s0.setPwmRange(new PwmControl.PwmRange(505, 2495));

        s1 = (ServoImplEx) hardwareMap.get(Servo.class, "rw");
        s1.setPwmRange(new PwmControl.PwmRange(505, 2495));
        s2 = (ServoImplEx) hardwareMap.get(Servo.class, "lw");
        s2.setPwmRange(new PwmControl.PwmRange(505, 2495));

        s3 = (ServoImplEx) hardwareMap.get(Servo.class, "claw");


    }

    @Override
    public void loop() {

        //winch

        if (gamepad1.b) {
            s2.setPosition(lw1);
            s1.setPosition(rw1);
        }

        if (gamepad1.x) {
            s2.setPosition(lw2);
            s1.setPosition(rw2);
        }

        if (gamepad1.a) {
            s2.setPosition(lwpickup);
            s1.setPosition(rwpickup);
            s0.setPosition(armpickup);
            s3.setPosition(clawopen);
        }

        if (gamepad1.y) {
            s2.setPosition(lwscore);
            s1.setPosition(rwscore);
            s0.setPosition(armscore);
        }

        if (gamepad1.options) {
            s3.setPosition(clawclose); 
        }




        telemetry.addData("Run time", getRuntime());
        telemetry.addData("1", "org/firstinspires/ftc/teamcode/test");
        telemetry.update();
    }
}