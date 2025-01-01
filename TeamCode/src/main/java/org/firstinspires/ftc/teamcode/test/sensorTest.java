package org.firstinspires.ftc.teamcode.test;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="sensorTest",  group="Test")
public class sensorTest extends OpMode {
    RevBlinkinLedDriver LED;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    DistanceSensor distanceSensor;
    ColorSensor colorSensor;
    public double distance;
    public double red;
    public double green;
    public double blue;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        LED.setPattern(pattern);
    }

    @Override
    public void loop() {

        distance = distanceSensor.getDistance(DistanceUnit.MM);
        red= colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        if ((red>=405&& red<=1631)&&(green>=235&&green<=900)&&(blue>=117&&blue<=524)) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }
        // Check for Blue Block
        else if ((green>=214&&green<=826)&&(blue>=500&&blue<=1828)&&(red<455)) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        }
        // Check for Yellow Block
        else if ((red>=690&&red<=2380)&&(green>=925&&green<=3100)&&(blue>=200&&blue<=790)) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        }
        else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        }
        LED.setPattern(pattern);


        telemetry.addData("Run time", getRuntime());
        telemetry.addData("Distance", distance);
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
        telemetry.update();
    }
}