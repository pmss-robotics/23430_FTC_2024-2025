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
public class IntakeSlidesSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;
    ServoImplEx intakeSlideL; // in order of precedence
    ServoImplEx intakeSlideR;
    // wrist moves hand and finger along an axis, wrist just moves fingers, etc.

    public static double F_target = 120; // in degrees
    public static double position = 120;

    private States.IntakeExtension currentSlidesState;
    public static int pIntake = 200;
    public static int pHome = 110;

    public IntakeSlidesSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        intakeSlideL = hardwareMap.get(ServoImplEx.class, "intakeSlideL");

        // expand the range of the servo beyond the default for control/expansion hubs
        // test
        intakeSlideL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSlideL.setPosition(scale(F_target));

        intakeSlideR = hardwareMap.get(ServoImplEx.class, "intakeSlideR");
        intakeSlideR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSlideR.setDirection(Servo.Direction.REVERSE);
        intakeSlideR.setPosition(scale(F_target));


        currentSlidesState = States.IntakeExtension.home;
    }

    public States.IntakeExtension getCurrentIntExState() {
        return currentSlidesState;
    }

    public boolean isExtended() {
        return currentSlidesState == States.IntakeExtension.intake;
    }

    public void incrementPosition(double increment) {
        position = MathUtils.clamp(position + increment, pHome, pIntake);
        intakeSlideL.setPosition(scale(position));
        intakeSlideR.setPosition(scale(position));
        currentSlidesState = States.IntakeExtension.middle;
    }

    public void toggleIntakeSlidesState() {
        switch (currentSlidesState) {
            case middle:
            case intake:
                intakeSlideL.setPosition(pHome);
                intakeSlideR.setPosition(pHome);
                currentSlidesState = States.IntakeExtension.home;
                break;
            case home:
                intakeSlideL.setPosition(scale(pIntake));
                intakeSlideR.setPosition(scale(pIntake));
                currentSlidesState = States.IntakeExtension.intake;
                break;
        }
    }

    public void holdPosition () {
        intakeSlideL.setPosition(scale(position));
        intakeSlideR.setPosition(scale(position));
    }

    public void setIntakeSlidesState(States.IntakeExtension state) {
        currentSlidesState = state;
        switch (currentSlidesState) {
            case home:
                intakeSlideL.setPosition(pHome);
                intakeSlideR.setPosition(pHome);
                break;
            case intake:
                intakeSlideL.setPosition(scale(pIntake));
                intakeSlideR.setPosition(scale(pIntake));
                break;
        }
    }

    @Override
    public void periodic() {
        telemetry.addData("intake slide position", intakeSlideL.getPosition());

    }

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

}