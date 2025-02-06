package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class IntakeSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;
    ServoImplEx wrist, arm, claw, wristR;
    // wrist rotates intake and spinners are rollers

    public static double W_target = 270; // in degrees
    public static double position = 270;
    private States.Intake currentIntakeState;

    public static int pHome = 262, pStart = 0, pIntake = 21, pTransfer = 190; // in degrees
    public static int wMin = 21, wMax = 262;

    public static int wHome = 0, wTransfer = 0, wIntake = 0, wMiddle = 0;
    public static int aHome = 0, aTransfer = 0, aIntake = 0, aMiddle = 0;
    public static int cOpen = 0, cClosed = 0;
    public static int wPosition = 0, wRotation = 0, aPosition;
    public static boolean intakeOpen = false;
    public static int rHome = 0, rMax = 0, rMin = 0;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;

        wrist = hardwareMap.get(ServoImplEx.class, "wristL");
        wristR = hardwareMap.get(ServoImplEx.class, "wristR");
        claw = hardwareMap.get(ServoImplEx.class, "intakeClaw");
        // expand the range of the servo beyond the default for control/expansion hubs
        // test
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wristR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));



        wrist.setPosition(scale(wHome));
        wPosition = wHome;
        wristR.setPosition(scale(rHome));
        wRotation = rHome;
        arm.setPosition(scale(aHome));
        aPosition = aHome;

        currentIntakeState = States.Intake.home;
    }

    public States.Intake getCurrentIntakeState() {
        return currentIntakeState;
    }

    public void toggleIntakeState() {
        switch (currentIntakeState) {
            case middle:
                wrist.setPosition(scale(wIntake));
                wPosition = wIntake;
                arm.setPosition(scale(aIntake));
                aPosition = aIntake;
                currentIntakeState = States.Intake.intake;
                break;
            case intake:
                wrist.setPosition(scale(wHome));
                wPosition = wHome;
                arm.setPosition(scale(aHome));
                aPosition = aHome;
                currentIntakeState = States.Intake.home;
                break;
            case home:
            case transfer:
                wrist.setPosition(scale(wMiddle));
                wPosition = wMiddle;
                arm.setPosition(scale(aMiddle));
                aPosition = aMiddle;
                currentIntakeState = States.Intake.middle;
                break;
        }
    }

    public void setIntakeState(States.Intake state) {
        currentIntakeState = state;
        switch (currentIntakeState) {
            case home:
                wrist.setPosition(scale(wHome));
                wPosition = wHome;
                arm.setPosition(scale(aHome));
                aPosition = aHome;
                break;
            case intake:
                wrist.setPosition(scale(wIntake));
                wPosition = wIntake;
                arm.setPosition(scale(aIntake));
                aPosition = aIntake;
                break;
            case transfer:
                wrist.setPosition(scale(wTransfer));
                wPosition = wTransfer;
                arm.setPosition(scale(aTransfer));
                aPosition = aTransfer;
                break;
            case middle:
                wrist.setPosition(scale(wMiddle));
                wPosition = wMiddle;
                arm.setPosition(scale(aMiddle));
                aPosition = aMiddle;
                break;
        }
    }

    public void toggleIntakeClaw () {
        if (intakeOpen) {
            claw.setPosition(scale(cClosed));
            intakeOpen = false;
        } else {
            claw.setPosition(scale(cOpen));
            intakeOpen = true;
        }
    }

    public void openIntakeClaw () {
        claw.setPosition(scale(cOpen));
        intakeOpen = true;
    }
    public void closeIntakeClaw () {
        claw.setPosition(scale(cClosed));
        intakeOpen = false;
    }

    public void incrementPosition(double increment) {
        position = MathUtils.clamp(position + increment, wMin, wMax);
        wrist.setPosition(scale(position));
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
        IntakeSubsystem.position = position;
    }



    @Override
    public void periodic() {
        telemetry.addData("intake wrist position", wrist.getPosition());
    }

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

}
