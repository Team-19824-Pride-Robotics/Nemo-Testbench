package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;

public class Limelight {

    private String limelightIP = "http://172.29.0.29"; // Default Limelight IP address for FTC
    private Map<Integer, String> pipelineColors;

    public Limelight(HardwareMap hardwareMap, String limelightName) {
        // Map pipeline numbers to their corresponding colors
        pipelineColors = new HashMap<>();
        pipelineColors.put(2, "Yellow");
        pipelineColors.put(3, "Blue");
    }

    /**
     * Detects the color currently in view based on the active pipeline.
     *
     * @return The detected color as a String (\"Blue\" or \"Yellow\"), or \"Unknown\" if no match.
     */
    public String detectColor() {
        // Retrieve the active pipeline from Limelight
        int activePipeline = getActivePipeline();

        // Return the color corresponding to the active pipeline
        return pipelineColors.getOrDefault(activePipeline, "Unknown");
    }

    /**
     * Retrieves the currently active pipeline from Limelight via HTTP.
     *
     * @return The active pipeline number, or 0 if the request fails.
     */
    private int getActivePipeline() {
        try {
            URL url = new URL(limelightIP + "/pipeline"); // Endpoint for pipeline info
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setConnectTimeout(1000);
            connection.setReadTimeout(1000);

            if (connection.getResponseCode() == 200) {
                InputStreamReader reader = new InputStreamReader(connection.getInputStream());
                int pipeline = Integer.parseInt(new java.util.Scanner(reader).useDelimiter("\\A").next());
                connection.disconnect();
                return pipeline;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return 0; // Default to 0 if the request fails
    }

    /**
     * Sets the active pipeline for the Limelight camera via HTTP.
     *
     * @param pipeline The pipeline number to set.
     */
    public void setPipeline(int pipeline) {
        try {
            URL url = new URL(limelightIP + "/setpipeline?number=" + pipeline); // Endpoint to set pipeline
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setConnectTimeout(1000);
            connection.setReadTimeout(1000);
            connection.getResponseCode(); // Trigger the request
            connection.disconnect();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
