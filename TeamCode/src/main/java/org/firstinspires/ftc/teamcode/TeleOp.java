package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.States;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    // probably need to change later.
    public static double servoIncrement = 7;
    public static double intakeSlideIncrement = 4;
    public static double servoSpeed = 1;
    public static double driveSpeed = 1;
    public static double fast = 1;
    public static double slow = 0.5;
    public static double rotationSpeed = 1;
    public static double wristStart = 0.5;
    public static double bucketStart = 0.636;
    public static double outtakeResetPower = 0.6;
    public static double ascentTiltPower = 0.2;
    public static double intakeSlidePowerO = 0.5;
    public static double intakeSlidePowerI = 0.7;

    States.Global currentState = States.Global.home;

    GamepadEx driver1, driver2;
    DriveSubsystem drive;
    OuttakeSlidesSubsystem outtakeSlides;
    IntakeSlidesSubsystem intakeSlides;
    VisionSubsystem vision;
    @Override
    public void initialize() {
        // data sent to telemetry shows up on dashboard and driverGamepad station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        // GamepadEx wraps gamepad 1 or 2 for easier implementations of more complex key bindings
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        // The driveSubsystem wraps Roadrunner's MecanumDrive to combine with Commands.
        //TODO check if this works
        DriveSubsystem drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(0, 0, Math.PI)), telemetry);
        // The driveCommand uses methods defined in the DriveSubsystem to create behaviour.
        // we're passing in methods to get values instead of straight values because it avoids
        // disturbing the structure of the CommandOpMode. The aim is to define bindings in this
        // initialize() method through Commands and these will be looped and acted in the (hidden)
        // run() loop.
        driveSpeed = fast;
        // macros to bring thing up and down
        // intake extenstion
        // outtake macro positions
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver1.getLeftX()*driveSpeed,
                () -> driver1.getLeftY()*driveSpeed,
                () -> -driver1.getRightX()*driveSpeed,
                true);

        outtakeSlides = new OuttakeSlidesSubsystem(hardwareMap, telemetry);
        outtakeSlides.setDefaultCommand(new RunCommand(outtakeSlides::holdPosition, outtakeSlides));

        intakeSlides = new IntakeSlidesSubsystem(hardwareMap, telemetry);
        intakeSlides.setDefaultCommand(new RunCommand(() -> intakeSlides.holdPosition(), intakeSlides));
/*      try {
            vision = new VisionSubsystem(hardwareMap, telemetry);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
*/
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap, telemetry);

        // reset everything, probably unnecessary
/*      SequentialCommandGroup returnHome = new SequentialCommandGroup(
                new InstantCommand(() -> intakeSlides.setIntakeSlidesState(States.IntakeExtension.home), intakeSlides),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home), outtakeSlides),
                new InstantCommand(() -> intake.setWristState(States.Intake.home), intake),
                swapState(States.Global.home)
        );
*/
        // IMU reset
        new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> drive.drive.pinpoint.resetPosAndIMU())
        );

        // slower driving
        new GamepadButton(driver1, GamepadKeys.Button.B).toggleWhenPressed(
                () -> driveSpeed = slow,
                () -> driveSpeed = fast
        );

        // intake rotation
        new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(
                        () -> intake.incrementPosition(servoIncrement),
                        intake
                ));
        new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(
                        () -> intake.incrementPosition(-servoIncrement),
                        intake
                ));

        // roller intake rotation
        new GamepadButton(driver2, GamepadKeys.Button.A).toggleWhenPressed(
                new InstantCommand(() -> intake.setPower(0.5+servoSpeed), intake),
                new InstantCommand(() -> intake.setPower(0.5), intake)
        );
        new GamepadButton(driver2, GamepadKeys.Button.B).toggleWhenPressed(
                new InstantCommand(() -> intake.setPower(1.2-servoSpeed), intake),
                new InstantCommand(() -> intake.setPower(0.5), intake)
        );



        new Trigger(() -> driver2.getLeftY() > 0.1 || driver2.getLeftY() < -0.1)
                .whileActiveContinuous(new InstantCommand (
                        () -> outtakeSlides.manual(driver2.getLeftY()*outtakeResetPower),
                        outtakeSlides
                ));

        new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> outtakeSlides.resetEncoder(), outtakeSlides)
        );

        // toggle outtake system
        new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(
                new ConditionalCommand(
                        new InstantCommand(() -> outtakeSlides.toggleBucket()),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> outtake.setWristState(States.Outtake.bucket)),
                                new WaitCommand(OuttakeSubsystem.dropTime),
                                new InstantCommand(() -> outtake.toggleWristState()),
                                new InstantCommand(() -> outtakeSlides.toggleBucket())
                        ),
                        () -> outtakeSlides.getCurrentOutExState() == States.OuttakeExtension.home
                )
        );
        new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> outtakeSlides.toggleSpecimen())
        );
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> outtake.toggleSpecimenOutput())
        );

        // horizontal extension

        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                .whileActiveContinuous(new InstantCommand (
                        () -> intakeSlides.manual(driver2.getRightX()*intakeSlidePowerI),
                        intakeSlides
                ));
        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whileActiveContinuous(new InstantCommand (
                        () -> intakeSlides.manual(driver2.getRightX()*intakeSlidePowerO),
                        intakeSlides
                ));

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
