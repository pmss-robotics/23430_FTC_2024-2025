package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

@Autonomous(name = "Example VisionPortal OpMode")
public class visiontestOp extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private AutoAlignProcessor sampleDetection;

    @Override
    public void runOpMode() {

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        sampleDetection = new AutoAlignProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Logi C270 HD WebCam"))
                .addProcessors(sampleDetection)
                .setCameraResolution(new Size(640, 480))
                //.setStreamFormat(VisionPortal.StreamFormat.MJPEG) // worse compression than the default but faster
                .build();



        // Wait for the DS start button to be touched.``
        waitForStart();


        while(opModeIsActive()) {
            getSampleAngle();
            telemetry.update();
        }

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();
    }

    public double getSampleAngle() {
        double angle = 0;
        AutoAlignProcessor.AnalyzedStone closest = null;
        List<AutoAlignProcessor.AnalyzedStone> detections = sampleDetection.getDetectedStones();
        for ( AutoAlignProcessor.AnalyzedStone sample : detections) {
            // find the sample closest to the claw
            telemetry.addData("Sample: " + detections.indexOf(sample), " "+ sample.color);
            telemetry.addData("Sample: " + detections.indexOf(sample), " " + (180- sample.angle));

            String x = Arrays.toString(sample.tvec.get(0,0));
            String y = Arrays.toString(sample.tvec.get(1,0));
            String z = Arrays.toString(sample.tvec.get(2,0));

            telemetry.addData("Sample: " + detections.indexOf(sample), "x " + x);
            telemetry.addData("Sample: " + detections.indexOf(sample), "y " + y);
            telemetry.addData("Sample: " + detections.indexOf(sample), "z " + z);
        }
        if(Objects.isNull(closest)) return 666;
        return angle;
    }
}
