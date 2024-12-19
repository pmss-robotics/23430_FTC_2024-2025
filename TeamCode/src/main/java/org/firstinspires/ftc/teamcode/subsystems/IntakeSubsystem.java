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

import java.util.function.DoubleSupplier;
import java.util.Objects;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class IntakeSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;
    ServoImplEx wrist, spinnerL, spinnerR;
    // wrist rotates intake and spinners are rollers

    public static double W_target = 50; // in degrees
    public static double position = 50;
    private States.Intake currentIntakeState;

    public static int pHome = 0, pStart = 0, pIntake = 0, pTransfer = 0; // in degrees
    public static int wMin = 50, wMax = 215;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        spinnerL = hardwareMap.get(ServoImplEx.class, "spinnerL");
        spinnerR = hardwareMap.get(ServoImplEx.class, "spinnerR");

        // expand the range of the servo beyond the default for control/expansion hubs
        // test
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
/*      spinnerL.setPwmRange(new PwmControl.PwmRange(500, 2500)); // do we need this?
        spinnerR.setPwmRange(new PwmControl.PwmRange(500, 2500));
*/      spinnerR.setDirection(Servo.Direction.REVERSE);

        wrist.setPosition(scale(W_target));
        position = W_target;

        currentIntakeState = States.Intake.home;
    }

    public States.Intake getCurrentIntakeState() {
        return currentIntakeState;
    }

    public void toggleWristState() {
        switch (currentIntakeState) {
            case transfer:
            case home:
                wrist.setPosition(scale(pIntake));
                position = pIntake;
                currentIntakeState = States.Intake.intake;
                break;
            case intake:
                wrist.setPosition(scale(pHome));
                position = pHome;
                currentIntakeState = States.Intake.home;
                break;
        }
    }

    public void setWristState(States.Intake state) {
        currentIntakeState = state;
        switch (currentIntakeState) {
            case home:
                wrist.setPosition(scale(pHome));
                position = pHome;
                break;
            case intake:
                wrist.setPosition(scale(pIntake));
                position = pIntake;
                break;
            case transfer:
                wrist.setPosition(scale(pTransfer));
                position = pTransfer;
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

    // To-do: continuous rotation method
    public void setPower(DoubleSupplier power) {
        spinnerL.setPosition(power.getAsDouble());
        spinnerR.setPosition(power.getAsDouble());
    }

    public void setPower(double power) {
        spinnerL.setPosition(power);
        spinnerR.setPosition(power);

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
