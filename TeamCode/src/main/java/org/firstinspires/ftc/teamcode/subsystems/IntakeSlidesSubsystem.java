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
    private ServoImplEx intakeSlideL; // in order of precedence
    private ServoImplEx intakeSlideR;
    private DcMotorEx hExtension;
    private VoltageSensor voltageSensor;
    // wrist moves hand and finger along an axis, wrist just moves fingers, etc.

    public static double F_target = 125; // in degrees
    public static double position = 125;

    private States.IntakeExtension currentSlidesState;
    public static int pIntake = 210;
    public static int pHome = 110;

    public static int target = 0;

    // public static double P = 0.0000001, I = 0, D = 0; // p: 0.021, i: 0.003

    // public PIDController pidController = new PIDController(P,I,D);


    public IntakeSlidesSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        intakeSlideL = hardwareMap.get(ServoImplEx.class, "intakeSlideL");
        intakeSlideL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSlideL.setPosition(scale(F_target));

        intakeSlideR = hardwareMap.get(ServoImplEx.class, "intakeSlideR");
        intakeSlideR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSlideR.setDirection(Servo.Direction.REVERSE);
        intakeSlideR.setPosition(scale(F_target));

        hExtension = hardwareMap.get(DcMotorEx.class, "intakeSlides");
        hExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

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
        hExtension.setPower(0);
    }

    /*
    private double calculate() {
        pidController.setPID(P,I,D);
        int current = hExtension.getCurrentPosition();

        double power = pidController.calculate(current, target);
        power /= voltageSensor.getVoltage();

        telemetry.addData("HExtension Power:", power);
        return power;
    }
     */

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

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

}