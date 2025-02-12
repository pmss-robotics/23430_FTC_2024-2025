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
public class IntakeSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;
    ServoImplEx armL, armR, claw, wrist;
    // wrist rotates intake and spinners are rollers

    public static double W_target = 270; // in degrees
    public static double position = 270;
    private States.Intake currentIntakeState;

    public static int wMin = 0, wMax = 200;

    public static int wHome = 25, wTransfer = 25, wIntake = 200, wMiddle = 160;
    public static int cOpen = 300, cClosed = 255;
    public static double wPosition = 25, wRotation = 150;
    public static boolean intakeOpen = false;
    public static int rRange = 80;
    public static int rHome = 110, rMax = 190, rMin = 30;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;

        armL = hardwareMap.get(ServoImplEx.class, "iArmL");
        armR = hardwareMap.get(ServoImplEx.class, "iArmR");
        wrist = hardwareMap.get(ServoImplEx.class, "iWrist");
        claw = hardwareMap.get(ServoImplEx.class, "iClaw");
        // expand the range of the servo beyond the default for control/expansion hubs
        // test
        armL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        armR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        armR.setDirection(Servo.Direction.REVERSE);


        armL.setPosition(scale(wHome));
        wPosition = wHome;
        wrist.setPosition(scale(rHome));
        wRotation = rHome;
        armR.setPosition(scale(wHome));
        claw.setPosition(scale(cClosed));

        currentIntakeState = States.Intake.home;
    }

    public States.Intake getCurrentIntakeState() {
        return currentIntakeState;
    }

    public void toggleIntakeState() {
        switch (currentIntakeState) {
            case middle:
                armL.setPosition(scale(wIntake));
                wPosition = wIntake;
                armR.setPosition(scale(wIntake));
                currentIntakeState = States.Intake.intake;
                break;
            case intake:
                armL.setPosition(scale(wTransfer));
                wPosition = wTransfer;
                armR.setPosition(scale(wTransfer));
                wrist.setPosition(scale(rHome));
                wRotation = rHome;
                currentIntakeState = States.Intake.transfer;
                break;
            case home:
            case transfer:
                armL.setPosition(scale(wMiddle));
                wPosition = wMiddle;
                armR.setPosition(scale(wMiddle));
                currentIntakeState = States.Intake.middle;
                break;
        }
    }

    public void setIntakeState(States.Intake state) {
        currentIntakeState = state;
        switch (currentIntakeState) {
            case home:
                armL.setPosition(scale(wHome));
                wPosition = wHome;
                armR.setPosition(scale(wHome));
                break;
            case intake:
                armL.setPosition(scale(wIntake));
                wPosition = wIntake;
                armR.setPosition(scale(wIntake));
                break;
            case transfer:
                armL.setPosition(scale(wTransfer));
                wPosition = wTransfer;
                armR.setPosition(scale(wTransfer));
                wrist.setPosition(scale(rHome));
                wRotation = rHome;
                break;
            case middle:
                armL.setPosition(scale(wMiddle));
                wPosition = wMiddle;
                armR.setPosition(scale(wMiddle));
                break;
        }
    }

    public void toggleIntakeClaw() {
        if (intakeOpen) {
            claw.setPosition(scale(cClosed));
            intakeOpen = false;
        } else {
            claw.setPosition(scale(cOpen));
            intakeOpen = true;
        }
    }

    public void openIntakeClaw() {
        claw.setPosition(scale(cOpen));
        intakeOpen = true;
    }
    public void closeIntakeClaw() {
        claw.setPosition(scale(cClosed));
        intakeOpen = false;
    }

    public void incrementPosition(double increment) {
        position = MathUtils.clamp(position + increment, wMin, wMax);
        armL.setPosition(scale(position));
        armR.setPosition(scale(position));
    }

    public void setWristPosition(double position) {
        armL.setPosition(scale(position));
        armR.setPosition(scale(position));
        IntakeSubsystem.position = position;
    }

    public void rotateClaw(double increment) {
        wRotation = MathUtils.clamp(wRotation + increment, rMin, rMax);
        wrist.setPosition(scale(wRotation));
    }



    @Override
    public void periodic() {
        telemetry.addData("intake wrist position", armL.getPosition());
    }

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

}
