package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystem.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.bucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.liftSubsystem;

@Config
@TeleOp(name="basicTeleop")
public class basicTeleop extends OpMode {
    private LinkageSubsystem linkage;
    private liftSubsystem lift;
    private intakeSubsystem intake;
    private bucketSubsystem bucket;
    private clawSubsystem claw;
    private armSubsystem arm;

    //arm

    //wrist
    ServoImplEx rw;
    ServoImplEx lw;



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        //wrist
        rw = (ServoImplEx) hardwareMap.get(Servo.class, "rw");
        rw.setPwmRange(new PwmControl.PwmRange(505, 2495));
        lw = (ServoImplEx) hardwareMap.get(Servo.class, "lw");
        lw.setPwmRange(new PwmControl.PwmRange(505, 2495));


    }

    @Override
    public void loop() {
          //////////////////////////
         /// Gamepad 1 controls ///
        //////////////////////////


          //////////////////////////
         /// Gamepad 2 controls ///
        //////////////////////////

        //linkage control
        if (gamepad2.b) {
            linkage.enableStickControl(0.13, 0.2);
        } else if (gamepad2.x) {
            linkage.disableStickControl();
        }
        if (linkage.isStickControlEnabled()) {
            double stickY = Math.max(-gamepad2.left_stick_y, 0);
            linkage.updateServoPositions(stickY);
        }

        //lift control
        if (gamepad2.back) {
            lift.pickup();
            arm.armPickup();
        }
        if (gamepad2.dpad_up) {
            lift.bucketHigh();
            arm.armSample();
        }
        if (gamepad2.dpad_down) {
            lift.bucketLow();
            arm.armSample();
        }
        if (gamepad2.dpad_right) {
            lift.barHigh();
            arm.armSpecimen();
        }
        if (gamepad2.dpad_left) {
            lift.barLow();
            arm.armSpecimen();
        }

        //intake control
        if (gamepad2.left_trigger > .1 || gamepad2.right_trigger > .1) {
            intakeSubsystem.intakeIn = Math.pow(gamepad2.left_trigger, 3);
            intakeSubsystem.intakeOut = Math.pow(-gamepad2.right_trigger,3);
            intake.intakeSetPower();
        }
        else{
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
        if (gamepad2.left_bumper){
            claw.clawClose();
        }

        lift.update();
        intake.update();
        bucket.update();
        claw.update();
        arm.update();


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

        telemetry.update();

    }
}