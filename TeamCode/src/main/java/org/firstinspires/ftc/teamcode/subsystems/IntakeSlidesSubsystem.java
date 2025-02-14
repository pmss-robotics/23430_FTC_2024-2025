package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.Objects;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class IntakeSlidesSubsystem extends SubsystemBase {

    // declare hardware here
    private Telemetry telemetry;
    // private ServoImplEx intakeSlideL; // in order of precedence
    // private ServoImplEx intakeSlideR;
    public DcMotorEx hExtension;
    private VoltageSensor voltageSensor;
    // wrist moves hand and finger along an axis, wrist just moves fingers, etc.

    public static double F_target = 125; // in degrees
    public static double position = 125;

    private States.IntakeExtension currentSlidesState;
    public static int pIntake = 210;
    public static int pHome = 110;

    public static int target = 0;
    public static int targetMax = 150, targetMin = 0;

    public static double P = 0.025, I = 0, D = 0.002; // TODO implement and tune, figure out how to get the slides to instantly stop when button is let go

    public PIDController pidController;


    public IntakeSlidesSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        /*
        intakeSlideL = hardwareMap.get(ServoImplEx.class, "intakeSlideL");
        intakeSlideL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSlideL.setPosition(scale(F_target));
         */

        /*
        intakeSlideR = hardwareMap.get(ServoImplEx.class, "intakeSlideR");
        intakeSlideR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSlideR.setDirection(Servo.Direction.REVERSE);
        intakeSlideR.setPosition(scale(F_target));
         */

        hExtension = hardwareMap.get(DcMotorEx.class, "intakeSlides");
        hExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        pidController = new PIDController(P,I,D);

        target = 0;

        currentSlidesState = States.IntakeExtension.home;
    }

    @Override
    public void periodic() {
        if (Objects.nonNull(hExtension.getCurrentPosition())) {
            telemetry.addData("HExtension Target: ", target);
            telemetry.addData("HExtension Pos: ", hExtension.getCurrentPosition());
            telemetry.update();
        }
    }

    public void holdPosition() {
        hExtension.setPower(calculate());
    }

    public void intakeExtension (double power) {
        if (power > 0) {
            target = targetMax;
            hExtension.setPower(calculate()*power+((targetMax)-hExtension.getCurrentPosition())/1200);
        } else if (power < 0) {
            target = targetMin;
            hExtension.setPower(-calculate()*power-0.2);
        } else {
            target = hExtension.getCurrentPosition();
            holdPosition();
        }
    }

    public void resetEncoder() {
        hExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        target = 0;
    }

    public void intakePosition (int pos) { target = pos; }

    public void retract () {
        target = -250;
    }

    public void intakeOut () {
        target = targetMax;
    }

    public void intakeIn () {
        target = targetMin;
    }

    public void resetTarget () {
        target = hExtension.getCurrentPosition();
    }

    private double calculate() {
        pidController.setPID(P,I,D);
        int current = hExtension.getCurrentPosition();

        double power = pidController.calculate(current, target);
        power /= voltageSensor.getVoltage();
        if (current < target-10) {
            power+= ((double)  ((targetMax) - hExtension.getCurrentPosition()-25) /360)*((double)  ((targetMax) - hExtension.getCurrentPosition()-25) /360);
        } else if (current > target+10){
            power -= 0.05;
        }
        telemetry.addData("HExtension Power:", power);
        return power;
    }


    public void manual(double power) {
        hExtension.setPower(power);
        target = hExtension.getCurrentPosition();
    }

    public States.IntakeExtension getCurrentIntExState() {
        return currentSlidesState;
    }

    public boolean isExtended() {
        return currentSlidesState == States.IntakeExtension.intake;
    }

    /*
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
     */

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

}