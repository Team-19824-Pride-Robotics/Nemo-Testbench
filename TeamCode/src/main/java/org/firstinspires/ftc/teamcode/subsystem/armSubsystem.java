package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config

public class armSubsystem {

    private final ServoImplEx arm;
    private final AnalogInput encoderArm;
    public static double armPickup = .24;

    public static double armSample = .7;
    public static double armSpecimen = .88;

    public double armTargetPosition =.24;

    public armSubsystem(HardwareMap hardwareMap) {
        arm = hardwareMap.get(ServoImplEx.class, "arm");
        arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        encoderArm = hardwareMap.get(AnalogInput.class, "eArm");

    }

    public void init() {
        arm.setPosition(armPickup);
    }

    public void armPickup() {
        armTargetPosition = armPickup;
    }

    public void armSample(){
        armTargetPosition = armSample;
    }
    public void armSpecimen(){
        armTargetPosition = armSpecimen;
    }
    public double getArmPosition() {
        return armTargetPosition;
    }
    public double getArmEncoderPosition() {
        return encoderArm.getVoltage() / 3.3 * 360;
    }
    public void update() {
        arm.setPosition(armTargetPosition);
    }

}
