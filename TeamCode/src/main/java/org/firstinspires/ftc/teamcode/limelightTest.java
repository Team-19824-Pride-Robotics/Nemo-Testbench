package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "limelightTest")
public class limelightTest extends LinearOpMode {

    private Limelight3A limelight;
    private Limelight limelight3A;

    @Override
    public void runOpMode() {
        // Initialize the Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addLine("Limelight Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Detect the color
            String detectedColor = limelight3A.detectColor();

            // Display the detected color
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();

            // Delay for telemetry updates
            sleep(50);
        }
    }

}
