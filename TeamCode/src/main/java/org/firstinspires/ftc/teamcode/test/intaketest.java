package org.firstinspires.ftc.teamcode.test;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name="intakeTest",  group="Test")
public class intaketest extends OpMode {

    //pid
    ServoImplEx s1;

    DcMotorEx m1;

    public static double pospower =1;
    public static  double negpower = -1;

    public static double sup = .28;
    public static double sdown =.36;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = (ServoImplEx) hardwareMap.get(Servo.class, "bucket");
        s1.setPwmRange(new PwmControl.PwmRange(505, 2495));

        m1 = hardwareMap.get(DcMotorEx.class, "intake");
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m1.setDirection(DcMotorEx.Direction.REVERSE);



    }

    @Override
    public void loop() {

        //winch

        if (gamepad1.dpad_down) {
            s1.setPosition(sdown);
        }
        if (gamepad1.dpad_up) {
            s1.setPosition(sup);
        }
        if (gamepad1.x) {
            m1.setPower(pospower);
        }
        if (gamepad1.b){
            m1.setPower(negpower);
        }
        if (gamepad1.a){
            m1.setPower(0);
        }



        telemetry.addData("Run time", getRuntime());
        telemetry.addData("1", "org/firstinspires/ftc/teamcode/test");
        telemetry.addData("pos1", m1.getCurrentPosition());
        telemetry.addData("power1", m1.getPower());
        telemetry.update();
    }
}