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
import org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.teamcode.util.States;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class OuttakeSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;
    ServoImplEx clawL;
    ServoImplEx clawR;
    ServoImplEx armL;
    ServoImplEx armR;
    ServoImplEx claw;
    // two servos to move the arm, two to move the claw, one to open and close the claw

    public static double W_target = 170; // in degrees
    public static double position = 170;
    public static double cPosition = 0;
    public static double aPosition = 0;
    public static boolean clawOpen = false;

    private States.Outtake currentOuttakeState;

    public static int pHome = 170, pStart = 0, pBucket = 240, pSpecimen = 233, pAscent = 300; // in degrees
    public static int cHome = 0, cStart = 0, cBucket = 0, cSpecimen = 0, cTransfer = 0;
    public static int aHome = 0, aStart = 0, aBucket = 0, aSpecimen = 0, aTransfer = 0;
    public static int cOpen = 0, cClosed = 0;
    public static int wMin = 0, wMax = 0;
    public static int cMin = 0, cMax = 300;
    public static int aMin = 0, aMax = 300;
    public static int dropTime = 1000;

    public OuttakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        clawL = hardwareMap.get(ServoImplEx.class, "clawL");
        clawL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawL.setPosition(scale(cStart));

        clawR = hardwareMap.get(ServoImplEx.class, "clawR");
        clawR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawR.setDirection(Servo.Direction.REVERSE);
        clawR.setPosition(scale(cStart));

        armL = hardwareMap.get(ServoImplEx.class, "armL");
        armL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        armL.setPosition(scale(aStart));

        armR = hardwareMap.get(ServoImplEx.class, "armR");
        armR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        armR.setDirection(Servo.Direction.REVERSE);
        armR.setPosition(scale(aStart));

        claw = hardwareMap.get(ServoImplEx.class, "claw");
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        claw.setPosition(scale(cClosed));

        cPosition = cStart;
        aPosition = aStart;
        clawOpen = false;

        currentOuttakeState = States.Outtake.home;
    }

    public States.Outtake getCurrentOuttakeState() {
        return currentOuttakeState;
    }

    public void toggleOuttakeState() {
        switch (TeleOp.currentMode) {
            case sample:
                switch (currentOuttakeState) {
                    case home:
                        setClawPosition(scale(cBucket));
                        setArmPosition(scale(aBucket));
                        currentOuttakeState = States.Outtake.bucket;
                        break;
                    case specimen:
                    case bucket:
                        setClawPosition(scale(cHome));
                        setArmPosition(scale(aHome));
                        currentOuttakeState = States.Outtake.home;
                        break;
                }
                break;
            case specimen:
                switch (currentOuttakeState) {
                    case home:
                        setClawPosition(scale(cSpecimen));
                        setArmPosition(scale(aSpecimen));
                        currentOuttakeState = States.Outtake.specimen;
                        break;
                    case bucket:
                    case specimen:
                        setClawPosition(scale(cHome));
                        setArmPosition(scale(aHome));
                        currentOuttakeState = States.Outtake.home;
                        break;
                }
                break;
        }
    }


    public void toggleSpecimenOutput() {
        switch (currentOuttakeState) {
            case home:
                clawL.setPosition(scale(pSpecimen));
                clawR.setPosition(scale(pSpecimen));
                position = pSpecimen;
                currentOuttakeState = States.Outtake.specimen;
                break;
            case specimen:
                clawL.setPosition(scale(pHome));
                clawR.setPosition(scale(pHome));
                position = pHome;
                currentOuttakeState = States.Outtake.home;
                break;
        }
    }

    public void toggleAscentPos() {
        switch (currentOuttakeState) {
            case home:
                clawL.setPosition(scale(pAscent));
                clawR.setPosition(scale(pAscent));
                position = pAscent;
                currentOuttakeState = States.Outtake.ascent;
                break;
            case ascent:
                clawL.setPosition(scale(pHome));
                clawR.setPosition(scale(pHome));
                position = pHome;
                currentOuttakeState = States.Outtake.home;
                break;
        }
    }

    public void setOuttakeState(States.Outtake state) {
        currentOuttakeState = state;
        switch (currentOuttakeState) {
            case home:
                setClawPosition(scale(cHome));
                setArmPosition(scale(aHome));
                break;
            case bucket:
                setClawPosition(scale(cBucket));
                setArmPosition(scale(aBucket));
                break;
            case specimen:
                setClawPosition(scale(cSpecimen));
                setArmPosition(scale(aSpecimen));
                break;
        }
    }

    public void toggleClaw () {
        if (clawOpen) {
            claw.setPosition(scale(cClosed));
            clawOpen = false;
        } else {
            claw.setPosition(scale(cOpen));
            clawOpen = true;
        }
    }

    public void openClaw () {
        claw.setPosition(scale(cOpen));
        clawOpen = true;
    }
    public void closeClaw () {
        claw.setPosition(scale(cClosed));
        clawOpen = false;
    }

    public void incrementClawPosition(double increment) {
        cPosition = MathUtils.clamp(cPosition + increment, cMin, cMax);
        clawL.setPosition(scale(cPosition));
        clawR.setPosition(scale(cPosition));
    }

    public void incrementArmPosition(double increment) {
        aPosition = MathUtils.clamp(aPosition + increment, aMin, aMax);
        armL.setPosition(scale(aPosition));
        armR.setPosition(scale(aPosition));
    }

    public void setClawPosition(double position) {
        clawL.setPosition(position);
        clawR.setPosition(position);
        cPosition = position;
    }

    public void setArmPosition(double position) {
        armL.setPosition(position);
        armR.setPosition(position);
        aPosition = position;
    }

    @Override
    public void periodic() {
        telemetry.addData("outtake claw position: ", clawL.getPosition());
        telemetry.addData("outtake arm position: ", armL.getPosition());
        if (clawOpen) {
            telemetry.addData("outtake claw state: ", "Open");
        } else {
            telemetry.addData("outtake claw state: ", "Closed");
        }
    }

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

}
