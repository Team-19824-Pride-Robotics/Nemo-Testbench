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
@TeleOp(name="linkageTest2",  group="Test")
public class LinkageTest2 extends OpMode {

    private ServoImplEx s1, s2;
    public static double rl1 = 0.09, rl2 = 0.27;
    public static double ll1 = 0.14, ll2 = 0.45;

    private static final double SERVO1_MIN = 0.09, SERVO1_MAX = 0.27;
    private static final double SERVO2_MIN = 0.14, SERVO2_MAX = 0.45;
    private static final double SERVO_FULL_MIN = 0.01, SERVO_FULL_MAX = 0.99;

    private boolean stickControlEnabled = false;
    private double stickControlMin = SERVO1_MIN;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = initServo("rl", SERVO1_MIN);
        s2 = initServo("ll", SERVO2_MIN);
    }

    private ServoImplEx initServo(String name, double initialPosition) {
        ServoImplEx servo = (ServoImplEx) hardwareMap.get(Servo.class, name);
        servo.setPwmRange(new PwmControl.PwmRange(505, 2495));
        servo.setPosition(initialPosition);
        return servo;
    }

    @Override
    public void loop() {
        if (gamepad1.b) {
            enableStickControl(0.13, 0.2);
        } else if (gamepad1.x) {
            disableStickControl();
        }

        if (stickControlEnabled) {
            double stickY = Math.max(-gamepad1.left_stick_y, 0);
            setServoPositions(map(stickY, 0, 1, stickControlMin, SERVO1_MAX));
        }

        updateTelemetry();
    }

    private void enableStickControl(double min, double position) {
        stickControlEnabled = true;
        stickControlMin = min;
        setServoPositions(position);
    }

    private void disableStickControl() {
        stickControlEnabled = false;
        setServoPositions(SERVO1_MIN);
    }

    private void updateTelemetry() {
        telemetry.addData("Run time", getRuntime());
        telemetry.addData("Stick Control Enabled", stickControlEnabled);
        telemetry.addData("Stick Control Min", stickControlMin);
        telemetry.addData("Servo 1 Position", s1.getPosition());
        telemetry.addData("Servo 2 Position", s2.getPosition());
        telemetry.update();
    }

    private double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    private double mapServo2(double servo1Position) {
        return map(servo1Position, SERVO1_MIN, SERVO1_MAX, SERVO2_MIN, SERVO2_MAX);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void setServoPositions(double servo1Position) {
        s1.setPosition(clamp(servo1Position, SERVO_FULL_MIN, SERVO_FULL_MAX));
        s2.setPosition(clamp(mapServo2(servo1Position), SERVO_FULL_MIN, SERVO_FULL_MAX));
    }
}
