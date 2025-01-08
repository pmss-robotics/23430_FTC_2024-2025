package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class OuttakeSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;
    ServoImplEx bucketL;
    ServoImplEx bucketR;
    // wrist rotates intake and spinners are rollers

    public static double W_target = 165; // in degrees
    public static double position = 165;

    private States.Outtake currentOuttakeState;

    public static int pHome = 165, pStart = 0, pBucket = 240, pSpecimen = 225, pAscent = 300; // in degrees
    public static int wMin = 0, wMax = 0;
    public static int dropTime = 1000;

    public OuttakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        bucketL = hardwareMap.get(ServoImplEx.class, "bucketL");
        bucketL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        bucketL.setPosition(scale(W_target));

        bucketR = hardwareMap.get(ServoImplEx.class, "bucketR");
        bucketR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        bucketR.setDirection(Servo.Direction.REVERSE);
        bucketR.setPosition(scale(W_target));

        position = W_target;

        currentOuttakeState = States.Outtake.home;
    }

    public States.Outtake getCurrentOuttakeState() {
        return currentOuttakeState;
    }

    public void toggleWristState() {
        switch (currentOuttakeState) {
            case home:
                bucketL.setPosition(scale(pBucket));
                bucketR.setPosition(scale(pBucket));
                position = pBucket;
                currentOuttakeState = States.Outtake.bucket;
                break;
            case bucket:
                bucketL.setPosition(scale(pHome));
                bucketR.setPosition(scale(pHome));
                position = pHome;
                currentOuttakeState = States.Outtake.home;
                break;
        }
    }

    public void toggleSpecimenOutput() {
        switch (currentOuttakeState) {
            case home:
                bucketL.setPosition(scale(pSpecimen));
                bucketR.setPosition(scale(pSpecimen));
                position = pSpecimen;
                currentOuttakeState = States.Outtake.specimen;
                break;
            case specimen:
                bucketL.setPosition(scale(pHome));
                bucketR.setPosition(scale(pHome));
                position = pHome;
                currentOuttakeState = States.Outtake.home;
                break;
        }
    }

    public void toggleAscentPos() {
        switch (currentOuttakeState) {
            case home:
                bucketL.setPosition(scale(pAscent));
                bucketR.setPosition(scale(pAscent));
                position = pAscent;
                currentOuttakeState = States.Outtake.ascent;
                break;
            case ascent:
                bucketL.setPosition(scale(pHome));
                bucketR.setPosition(scale(pHome));
                position = pHome;
                currentOuttakeState = States.Outtake.home;
                break;
        }
    }

    public void setWristState(States.Outtake state) {
        currentOuttakeState = state;
        switch (currentOuttakeState) {
            case home:
                bucketL.setPosition(scale(pHome));
                bucketR.setPosition(scale(pHome));
                position = pHome;
                break;
            case bucket:
                bucketL.setPosition(scale(pBucket));
                bucketR.setPosition(scale(pBucket));
                position = pBucket;
                break;
        }
    }

    public void incrementPosition(double increment) {

        position = MathUtils.clamp(position + increment, wMin, wMax);
        bucketL.setPosition(scale(position));
        bucketR.setPosition(scale(position));
    }

    public void setPosition(double position) {
        bucketL.setPosition(position);
        bucketR.setPosition(position);
        IntakeSubsystem.position = position;
    }

    @Override
    public void periodic() {
        telemetry.addData("outtake wrist position", bucketL.getPosition());
    }

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

}
