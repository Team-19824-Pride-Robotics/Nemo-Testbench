package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.bucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.liftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.wristSubsystem;

import java.util.List;

@Config
@TeleOp(name="Teleop")
public class Teleop extends OpMode {
    private LinkageSubsystem linkage;
    private liftSubsystem lift;
    private intakeSubsystem intake;
    private bucketSubsystem bucket;
    private clawSubsystem claw;
    private armSubsystem arm;
    private wristSubsystem wrist;
    boolean dpad_up =false ;
    boolean dpad_down =false ;

    boolean dpad_right =false ;
    boolean dpad_left =false ;
    boolean pickup = true;
    boolean pickup2 =  false;
    boolean liftPickup = true;
    boolean intaking = false;

    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;

//sensor
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

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        linkage = new LinkageSubsystem(hardwareMap);
        linkage.init();
        lift = new liftSubsystem(hardwareMap);
        lift.init();
        intake = new intakeSubsystem(hardwareMap);
        intake.init();
        bucket = new bucketSubsystem(hardwareMap);
        bucket.init();
        claw = new clawSubsystem(hardwareMap);
        claw.init();
        arm = new armSubsystem(hardwareMap);
        arm.init();
        wrist = new wristSubsystem(hardwareMap);
        wrist.init();

        LED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        LED.setPattern(pattern);

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs){
            hub.clearBulkCache();
        }

        distance = distanceSensor.getDistance(DistanceUnit.MM);
        red= colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        //////////////////////////
        /// Gamepad 1 controls ///
        //////////////////////////


        //////////////////////////
        /// Gamepad 2 controls ///
        //////////////////////////

        //linkage control
        if (gamepad2.b) {
            linkage.enableStickControl(0.13, 0.2);
            arm.armPickup();
            liftPickup = false;
            intaking = true;
        } else if (gamepad2.x) {
            linkage.disableStickControl();
            pickup2= true;
            intaking = false;
        }
        if (linkage.getEncoderRlPosition() <= 48 && pickup2) {
            arm.armPickup2();
            pickup2 = false;
        }
        if (linkage.isStickControlEnabled()) {
            double stickY = Math.max(-gamepad2.left_stick_y, 0);
            linkage.updateServoPositions(stickY);
        }

        //lift control
        if (gamepad2.back) {
            wrist.wristPickup();
            arm.armPickup();
            pickup = true;
            liftPickup = true;
        }
        if (arm.getArmPosition() <= 115&& pickup) {
            lift.pickup();
            pickup = false;
        }
        if (lift.getLift1Position() <= 10 && liftPickup) {
            arm.armPickup2();
            liftPickup = false;
        }
        if (gamepad2.dpad_up) {
            dpad_up = true;
            lift.bucketHigh();
            arm.armSample();
            wrist.wristOut();
        }
        if (gamepad2.dpad_down) {
            dpad_down = true;
            lift.bucketLow();
            arm.armSample();
            wrist.wristOut();

        }
        if (gamepad2.dpad_right) {
            dpad_right=true;
            lift.barLow();
            arm.armSpecimen();
            wrist.wristOut();
        }
        if (gamepad2.dpad_left) {
            dpad_left = true;
            lift.barHigh();
            arm.armSpecimen();
            wrist.wristOut();
        }
        if ((arm.getArmEncoderPosition() >= 200 && dpad_right) || (arm.getArmEncoderPosition() >= 200 && dpad_left) || (arm.getArmEncoderPosition() >= 200 && dpad_up) || (arm.getArmEncoderPosition() >= 200 && dpad_down)) {
            wrist.wristScore();
            pickup = false;
            liftPickup = false;
            dpad_right = false;
            dpad_left = false;
            dpad_down = false;
            dpad_up = false;
        }

        //intake control
        if (gamepad2.left_trigger > .1 || gamepad2.right_trigger > .1) {
            intakeSubsystem.intakeIn = Math.pow(gamepad2.left_trigger, 3);
            intakeSubsystem.intakeOut = Math.pow(-gamepad2.right_trigger, 3);
            intake.intakeSetPower();
        } else {
            intake.intakeSetIdle();
        }

        //bucket control
        if (gamepad2.y) {
            bucket.bucketUp();
        }
        if (gamepad2.a) {
            bucket.bucketDown();
        }

        //claw control
        if (gamepad2.right_bumper) {
            claw.clawOpen();
        }
        if (gamepad2.left_bumper) {
            claw.clawClose();
        }

        lift.update();
        intake.update();
        bucket.update();
        claw.update();
        arm.update();
        wrist.update();

        //led
        if (intaking){
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
                pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
            }
        }
        if (!intaking){
            if (distance <=250&& distance>25){
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            }
            else if(distance<=25){
                pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
            }
            else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            }
        }

        LED.setPattern(pattern);

        telemetry.addData("Run time", getRuntime());
        //linkage
        telemetry.addData("Stick Control Enabled", linkage.isStickControlEnabled());
        telemetry.addData("Stick Control Min", linkage.getStickControlMin());
        telemetry.addData("RL Position", linkage.getServo1Position());
        telemetry.addData("LL Position", linkage.getServo2Position());
        telemetry.addData("Encoder RL Position", linkage.getEncoderRlPosition());
        telemetry.addData("Encoder LL Position", linkage.getEncoderLlPosition());
        //lift
        telemetry.addData("Lift Target", lift.getTarget());
        telemetry.addData("Lift Position 1", lift.getLift1Position());
        telemetry.addData("Lift Position 2", lift.getLift2Position());
        //intake
        telemetry.addData("intakePower", intake.getIntakePower());
        telemetry.addData("intakePowerin", intake.getIntakeIn());
        telemetry.addData("intakePowerout", intake.getIntakeOut());
        //bucket
        telemetry.addData("bucketTarget", bucket.getBucketPosition());
        telemetry.addData("bucketEncoder", bucket.getBucketEncoderPosition());
        //claw
        telemetry.addData("clawPosition", claw.getClawPosition());
        //arm
        telemetry.addData("armTarget", arm.getArmPosition());
        telemetry.addData("armEncoder", arm.getArmEncoderPosition());
        //wrist
        telemetry.addData("lwTarget", wrist.getlwTargetPosition());
        telemetry.addData("lwEncoder", wrist.getLwEncoderPosition());
        telemetry.addData("rwTarget", wrist.getRwTargetPosition());
        telemetry.addData("lwEncoder", wrist.getRwEncoderPosition());

        telemetry.addData("dpad", dpad_up);

        telemetry.addData("Distance", distance);

        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
        telemetry.update();
    }
}