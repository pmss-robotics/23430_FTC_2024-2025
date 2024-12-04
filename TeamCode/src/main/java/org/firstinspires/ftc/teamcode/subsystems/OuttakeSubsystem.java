package org.firstinspires.ftc.teamcode.subsystems;

import android.bluetooth.le.ScanSettings;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.Objects;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class OuttakeSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;
    ServoImplEx wrist;
    // wrist rotates intake and spinners are rollers

    public static double W_target = 260; // in degrees
    public static double position = 0;

    private States.Outtake currentOuttakeState;

    public static int pHome = 260, pStart = 0, pBucket = 150, pSpecimen = 170, pAscent = 200; // in degrees
    public static int wMin = 0, wMax = 0;
    public static int dropTime = 1000;

    public OuttakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        wrist = hardwareMap.get(ServoImplEx.class, "bucket");

        // expand the range of the servo beyond the default for control/expansion hubs
        // test
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));

        wrist.setPosition(scale(W_target));
        position = W_target;

        currentOuttakeState = States.Outtake.home;
    }

    public States.Outtake getCurrentOuttakeState() {
        return currentOuttakeState;
    }

    public void toggleWristState() {
        switch (currentOuttakeState) {
            case home:
                wrist.setPosition(scale(pBucket));
                position = pBucket;
                currentOuttakeState = States.Outtake.bucket;
                break;
            case bucket:
                wrist.setPosition(scale(pHome));
                position = pHome;
                currentOuttakeState = States.Outtake.home;
                break;
        }
    }

    public void toggleSpecimenOutput() {
        switch (currentOuttakeState) {
            case home:
                wrist.setPosition(scale(pSpecimen));
                position = pSpecimen;
                currentOuttakeState = States.Outtake.specimen;
                break;
            case specimen:
                wrist.setPosition(scale(pHome));
                position = pHome;
                currentOuttakeState = States.Outtake.home;
                break;
        }
    }

    public void toggleAscentPos() {
        switch (currentOuttakeState) {
            case home:
                wrist.setPosition(scale(pAscent));
                position = pAscent;
                currentOuttakeState = States.Outtake.ascent;
                break;
            case ascent:
                wrist.setPosition(scale(pHome));
                position = pHome;
                currentOuttakeState = States.Outtake.home;
                break;
        }
    }

    public void setWristState(States.Outtake state) {
        currentOuttakeState = state;
        switch (currentOuttakeState) {
            case home:
                wrist.setPosition(scale(pHome));
                position = pHome;
                break;
            case bucket:
                wrist.setPosition(scale(pBucket));
                position = pBucket;
                break;
        }
    }

    public void incrementPosition(double increment) {

        position = MathUtils.clamp(position + increment, wMin, wMax);
        wrist.setPosition(scale(position));
    }

    public void setPosition(double position) {
        wrist.setPosition(position);
        IntakeSubsystem.position = position;
    }

    @Override
    public void periodic() {
        telemetry.addData("outtake wrist position", wrist.getPosition());
    }

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

}
