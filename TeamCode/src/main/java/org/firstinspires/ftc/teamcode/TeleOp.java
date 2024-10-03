package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GenericContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GenericMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GenericPositionServoSubsystem;

//hello
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    public static double servoIncrement = 0.004;
    public static double servoSpeed = 1;
    public static double wristStart = 0.5;
    public static double bucketStart = 0.636;
    @Override
    public void initialize() {
        // data sent to telemetry shows up on dashboard and driverGamepad station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        // GamepadEx wraps gamepad 1 or 2 for easier implementations of more complex key bindings
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx tools = new GamepadEx(gamepad2);
        // The driveSubsystem wraps Roadrunner's MecanumDrive to combine with Commands.
        DriveSubsystem drive = new DriveSubsystem(new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)), telemetry);
        // The driveCommand uses methods defined in the DriveSubsystem to create behaviour.
        // we're passing in methods to get values instead of straight values because it avoids
        // disturbing the structure of the CommandOpMode. The aim is to define bindings in this
        // initialize() method through Commands and these will be looped and acted in the (hidden)
        // run() loop.

        // macros to bring thing up and down
        // intake extenstion
        // outtake macro positions
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver.getLeftX(),
                () -> -driver.getLeftY(),
                () -> -driver.getRightX(),
                false);

        GenericMotorSubsystem intakeSlides = new GenericMotorSubsystem(hardwareMap, telemetry, "intakeMotor");
        intakeSlides.setDefaultCommand(new RunCommand(
                () -> intakeSlides.setPower(tools.getRightY()),
                intakeSlides
        ));

        GenericMotorSubsystem elevator = new GenericMotorSubsystem(hardwareMap, telemetry, "elevatorMotor");
        elevator.setDefaultCommand(new RunCommand(
                () -> elevator.setPower(tools.getLeftY()),
                elevator
        ));

        GenericPositionServoSubsystem wrist = new GenericPositionServoSubsystem(hardwareMap, telemetry, "wrist", wristStart);
        wrist.setDefaultCommand(new RunCommand(() -> wrist.setPosition(wrist.position), wrist));
        new GamepadButton(tools, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(
                        () -> wrist.incrementPosition(-servoIncrement),
                        wrist
                ));
        new GamepadButton(tools, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(
                        () -> wrist.incrementPosition(servoIncrement),
                        wrist
                ));

        GenericPositionServoSubsystem bucket = new GenericPositionServoSubsystem(hardwareMap, telemetry, "bucket", bucketStart);
        bucket.setDefaultCommand(new RunCommand(
                () -> bucket.setPosition(bucket.position),
                bucket
        ));

        new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(
                        () -> bucket.incrementPosition(-servoIncrement),
                        bucket
                ));
        new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(
                        () -> bucket.incrementPosition(servoIncrement),
                        bucket
                ));


        GenericContinuousServoSubsystem spinner = new GenericContinuousServoSubsystem(hardwareMap, telemetry, "spinner");
        // to trigger you can do something similar to whats done in genericMotorSubsystem or...
        new GamepadButton(tools, GamepadKeys.Button.A).toggleWhenPressed(
                new InstantCommand(() -> spinner.setPower(servoSpeed)),
                new InstantCommand(() -> spinner.setPower(0.5))
                );
        new GamepadButton(tools, GamepadKeys.Button.B).toggleWhenPressed(
                new InstantCommand(() -> spinner.setPower(-servoSpeed)),
                new InstantCommand(() -> spinner.setPower(0.5))
        );


        schedule(new RunCommand(() -> {
            TelemetryPacket packet = new TelemetryPacket();
            Pose2d pose = drive.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y",pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }));
        schedule(driveCommand);
    }


}
