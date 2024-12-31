package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config

public class wristSubsystem {

    private final ServoImplEx rw, lw;
    private final AnalogInput encoderRw, encoderLw;

    public static double rwPickup = .34;
    public static double lwPickup = .38;

    public static double rwScore = .46;
    public static double lwScore = .5;


    public double rwTargetPosition;
    public double lwTargetPosition;

    public wristSubsystem(HardwareMap hardwareMap) {
        rw = hardwareMap.get(ServoImplEx.class, "rw");
        rw.setPwmRange(new PwmControl.PwmRange(505, 2495));
        encoderRw = hardwareMap.get(AnalogInput.class, "eRw");
        lw = hardwareMap.get(ServoImplEx.class, "lw");
        lw.setPwmRange(new PwmControl.PwmRange(505, 2495));
        encoderLw = hardwareMap.get(AnalogInput.class, "eLw");

    }

    public void init() {
        rw.setPosition(rwPickup);
        lw.setPosition(lwPickup);
    }

    public void wristPickup() {
        rwTargetPosition = rwPickup;
        lwTargetPosition = lwPickup;
    }
    public void wristScore() {
        rwTargetPosition = rwScore;
        lwTargetPosition = lwScore;
    }

    public double getRwTargetPosition() {
        return rwTargetPosition;
    }
    public double getlwTargetPosition() {
        return lwTargetPosition;
    }
    public double getRwEncoderPosition() {
        return encoderRw.getVoltage() / 3.3 * 360;
    }
    public double getLwEncoderPosition() {
        return encoderLw.getVoltage() / 3.3 * 360;
    }
    public void update() {
        rw.setPosition(rwTargetPosition);
        lw.setPosition(lwTargetPosition);
    }

}

