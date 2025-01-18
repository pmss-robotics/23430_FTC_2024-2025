package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RGBCycle extends SubsystemBase {
    private Servo rgbLight;
    private Telemetry telemetry;
    // Constructor to initialize the subsystem
    public RGBCycle(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        Servo rgbLight = hardwareMap.get(Servo.class, "rgbLigt");
    }

    /**
     * Set the light to a specific position.
     * @param position A value between 0.0 (off) and 1.0 (white).
     */
    public void setPosition(double position) {
        rgbLight.setPosition(position);
    }

    /**
     * Cycle the RGB light smoothly through all colors.
     * @param delayMs Delay between steps, controls speed of cycling.
     */
    public void cycleColors(int delayMs) {
        for (double position = 0.0; position <= 1.0; position += 0.01) {
            setPosition(position);

            // Pause for a short delay to allow smooth cycling
            try {
                Thread.sleep(delayMs);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}