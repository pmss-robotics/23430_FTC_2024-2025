package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.Objects;

@Config
public class AscentSubsystem extends SubsystemBase {

    private DcMotorEx ascent;
    private Telemetry telemetry;
    public static double P = 0, I = 0, D = 0; // p: 0.021, i: 0.003
    public static double kSpring = 0;
    public PIDController pidController;
    private VoltageSensor voltageSensor;
    public static int pHome = 0, pAscent = 0, pStart = 0;
    public static int target = 0;
    private States.Ascent currentState;

    public AscentSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        currentState = States.Ascent.home;

        ascent = hardwareMap.get(DcMotorEx.class, "ascent");
        ascent.setDirection(DcMotorSimple.Direction.REVERSE);
        ascent.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ascent.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascent.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        target = 0;

        pidController = new PIDController(P, I, D);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    @Override
    public void periodic() {
        if (Objects.nonNull(ascent.getCurrentPosition())) {
            telemetry.addData("Ascent Pos: ", ascent.getCurrentPosition());
            telemetry.update();
        }
    }

    public void holdPosition() {
        ascent.setPower(0);
    }

    public void moveTo(int target) {
        AscentSubsystem.target = target;
        holdPosition();
    }

    private double calculate() {
        pidController.setPID(P,I,D);
        int current = ascent.getCurrentPosition();

        double power = pidController.calculate(current, target) + kSpring;
        power /= voltageSensor.getVoltage();

        telemetry.addData("Ascent Power:", power);
        return power;
    }

    public void manual(double power) {
        ascent.setPower(power);
        target = ascent.getCurrentPosition();
    }

    public void toggleState() {
        switch (currentState) {
            case home:
                moveTo(pAscent);
                currentState = States.Ascent.ascent;
                break;
            case ascent:
                moveTo(pHome);
                currentState = States.Ascent.home;
                break;
        }
    }

    public void resetEncoder() {
        ascent.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascent.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        target = 0;
    }

    public void setState(States.Ascent state) {
        currentState = state;
        switch (currentState) {
            case home:
                moveTo(pHome);
                break;
            case ascent: //maybe change later so it works
                moveTo(pAscent);
                break;
            case start:
                moveTo(pStart);
                break;
        }
    }
}
