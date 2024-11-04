package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

// tuning guide: https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
@Config
public class OuttakeSlidesSubsystem extends SubsystemBase {

    private MotorGroup extensions;
    private Telemetry telemetry;
    public static double P = 0, I = 0, D = 0;
    public static double kSpring = 0;
    public static int pHome = 0, pSpecimen = 0, pBucket = 0, pStart = 0;
    public static int target = 0;
    public PIDController pidController;
    private VoltageSensor voltageSensor;
    private States.OuttakeExtension currentState;

    public OuttakeSlidesSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        currentState = States.OuttakeExtension.home;

        MotorEx leftExtension = new MotorEx(hardwareMap, "outtake left");
        MotorEx rightExtension = new MotorEx(hardwareMap, "outtake right");
        rightExtension.setInverted(true);
        extensions = new MotorGroup(leftExtension, rightExtension);
        extensions.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extensions.stopAndResetEncoder();

        pidController = new PIDController(P, I, D);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    @Override
    public void periodic() {
        telemetry.addData("Extension Target: ", target);
        telemetry.addData("Extension Pos: ", extensions.getCurrentPosition());
        telemetry.update();
    }

    public void holdPosition() {
        extensions.set(calculate());
    }

    public void moveTo(int target) {
        OuttakeSlidesSubsystem.target = target;
        holdPosition();
    }

    public void manual(double power) {
        extensions.set(calculate() + power);
        target = extensions.getCurrentPosition();
    }

    private double calculate() {
        pidController.setPID(P,I,D);
        int current = extensions.getCurrentPosition();

        double power = kSpring - pidController.calculate(current, target);
        // we are subtracting the PID since the springs are constantly trying to extend the arm
        power /= voltageSensor.getVoltage();

        telemetry.addData("Extension Power:", power);
        return power;
    }

    public States.OuttakeExtension getCurrentOutExState() {
        return currentState;
    }

    public void setState(States.OuttakeExtension state) {
        currentState = state;
        switch (currentState) {
            case bucket:
                moveTo(pBucket);
                break;
            case specimen: //maybe change later so it works
                moveTo(pSpecimen);
                break;
            case home:
                moveTo(pHome);
                break;
            case start:
                moveTo(pStart);
                break;
        }
    }
}
