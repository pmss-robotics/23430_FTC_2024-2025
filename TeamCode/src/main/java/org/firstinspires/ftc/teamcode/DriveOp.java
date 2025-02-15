package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.States;

//hello
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DriveOP", group = "TeleOp")
public class DriveOp extends CommandOpMode {
    // probably need to change later.
    public static double servoIncrement = 7;
    public static double servoSpeed = 1;
    public static double driveSpeed = 0.2;
    public static double rotationSpeed = 0.2;
    public static double wristStart = 0.5;
    public static double bucketStart = 0.636;
    States.Global currentState = States.Global.home;

    GamepadEx driver, tools;
    DriveSubsystem drive;
    /*    OuttakeSlidesSubsystem outtakeSlides;
        IntakeSlidesSubsystem intakeSlides;
        VisionSubsystem vision; */
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
        DriveSubsystem drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0)), telemetry);
        // The driveCommand uses methods defined in the DriveSubsystem to create behaviour.
        // we're passing in methods to get values instead of straight values because it avoids
        // disturbing the structure of the CommandOpMode. The aim is to define bindings in this
        // initialize() method through Commands and these will be looped and acted in the (hidden)
        // run() loop.

        // macros to bring thing up and down
        // intake extenstion
        // outtake macro positions
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver.getLeftX()*driveSpeed,
                () -> driver.getLeftY()*driveSpeed,
                () -> -driver.getRightX()*rotationSpeed,
                false);

//        outtakeSlides = new OuttakeSlidesSubsystem(hardwareMap, telemetry);
//        outtakeSlides.setDefaultCommand(new RunCommand(outtakeSlides::holdPosition, outtakeSlides));

//        intakeSlides = new IntakeSlidesSubsystem(hardwareMap, telemetry);
/*      try {
            vision = new VisionSubsystem(hardwareMap, telemetry);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
*/
//        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);

        //OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap, telemetry);

        // reset everything, probably unnecessary
/*      SequentialCommandGroup returnHome = new SequentialCommandGroup(
                new InstantCommand(() -> intakeSlides.setIntakeSlidesState(States.IntakeExtension.home), intakeSlides),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home), outtakeSlides),
                new InstantCommand(() -> intake.setWristState(States.Intake.home), intake),
                swapState(States.Global.home)
        );
*/
        // intake rotation
/*        new GamepadButton(tools, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(
                        () -> intake.incrementPosition(-servoIncrement),
                        intake
                ));
        new GamepadButton(tools, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(
                        () -> intake.incrementPosition(servoIncrement),
                        intake
                ));

        // roller intake rotation
        new GamepadButton(tools, GamepadKeys.Button.A).toggleWhenPressed(
                new InstantCommand(() -> intake.setPower(0.5+servoSpeed), intake),
                new InstantCommand(() -> intake.setPower(0.5), intake)
        );
        new GamepadButton(tools, GamepadKeys.Button.B).toggleWhenPressed(
                new InstantCommand(() -> intake.setPower(0.5-servoSpeed), intake),
                new InstantCommand(() -> intake.setPower(0.5), intake)
        );

        // toggle intake slides
        new GamepadButton(tools, GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> intakeSlides.toggleIntakeSlidesState())
        );

        // toggle outtake system
        new GamepadButton(tools, GamepadKeys.Button.Y).whenPressed(
                new ConditionalCommand(
                        new InstantCommand(() -> outtakeSlides.toggleState()),
                        new SequentialCommandGroup(
                                //new InstantCommand(() -> outtake.setWristState(States.Outtake.bucket)),
                                new WaitCommand(OuttakeSubsystem.dropTime),
                                //new InstantCommand(() -> outtake.toggleWristState()),
                                new InstantCommand(() -> outtakeSlides.toggleState())
                        ),
                        () -> outtakeSlides.getCurrentOutExState() == States.OuttakeExtension.home
                )
        );
*/
        // TODO transfer system

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

    public InstantCommand swapState(States.Global state) {
        return new InstantCommand(() -> currentState = state);
    }


}
