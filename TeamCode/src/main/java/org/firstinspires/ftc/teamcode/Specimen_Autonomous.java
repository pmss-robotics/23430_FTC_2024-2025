package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.stream.Collectors;
import java.util.stream.Stream;


@Config
@Autonomous(name="Red_Bucket_Auto", group="Auto")
public class Specimen_Autonomous extends CommandOpMode {
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DriveSubsystem drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(-38, -61.5,Math.PI/2)), telemetry);

        //auto pathing
        Action trajectoryAction = drive.actionBuilder(drive.getPose())
                .lineToYLinearHeading(-35, Math.PI*1.5)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(35, -35))
                .strafeTo(new Vector2d(35, -13))
                .splineToLinearHeading(new Pose2d(new Vector2d( 46, -13), Math.PI/2), Math.PI)
                .strafeTo(new Vector2d(46, -48))
                .strafeTo(new Vector2d(46, -13))
                .strafeTo(new Vector2d(55, -13))
                .strafeTo(new Vector2d(55, -48))
                .strafeTo(new Vector2d(55, -45))
                .splineToConstantHeading(new Vector2d(37, -55), -Math.PI/2)
                .strafeTo(new Vector2d(37, -59))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(12, -35), Math.PI*1.5)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(37, -55), Math.PI*.5)
                .strafeTo(new Vector2d(37, -59))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(8, -35), Math.PI*1.5)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(37, -55), Math.PI*.5)
                .strafeTo(new Vector2d(37, -59))
                .strafeToLinearHeading(new Vector2d(6, -35), Math.PI*1.5)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(45, -59), Math.PI*.5)
                .build();
        Command trajectory = new ActionCommand(trajectoryAction, Stream.of(drive).collect(Collectors.toSet()));

        OuttakeSlidesSubsystem outtakeSlides = new OuttakeSlidesSubsystem(hardwareMap, telemetry);
        outtakeSlides.setDefaultCommand(new RunCommand(outtakeSlides::holdPosition, outtakeSlides));
        IntakeSlidesSubsystem intakeSlides = new IntakeSlidesSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap, telemetry);

        waitForStart();

        //drop sample into high bucket
        Command bucket = new SequentialCommandGroup(
                new InstantCommand(() -> intake.setPosition(60), intake),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket), outtakeSlides),
                new InstantCommand(() -> outtake.setWristState(States.Outtake.bucket), outtake),
                new WaitCommand(OuttakeSubsystem.dropTime),
                new InstantCommand(() -> outtake.toggleWristState(), outtake),
                new InstantCommand(() -> outtakeSlides.toggleBucket(), outtakeSlides)
        );

        //intake a sample
        Command sample = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intakeSlides.setIntakeSlidesState(States.IntakeExtension.intake)),
                        new InstantCommand(() -> intake.setWristState(States.Intake.intake))
                ),
                new InstantCommand(() -> intake.setPower(0.5+1), intake),
                new WaitCommand(800),
                new InstantCommand(() -> intake.setPower(0.5), intake),
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.toggleWristState()),
                        new InstantCommand(() -> intakeSlides.toggleIntakeSlidesState())
                ),
                new InstantCommand(() -> intake.setPower(1-1), intake),
                new WaitCommand(800),
                new InstantCommand(() -> intake.setPower(0.5), intake)
        );

        //specimen cycle system
        //TODO adjust timing (everything will probably break if you just run this)
        Command auto = new ParallelCommandGroup(
                new SequentialCommandGroup(
                          new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                          new InstantCommand(() -> outtake.toggleSpecimenOutput()),
                          new WaitCommand(2000),
                          new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(13300),
                        new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(15180),
                        new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(18100),
                        new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(20140),
                        new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(23050),
                        new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(25090),
                        new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home))
                )
        );
        schedule(new ParallelCommandGroup(trajectory,auto));
        // TODO: create wrappers for trajectory following maybe possibly
        // this RunCommand Loop might be useless
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
    }
}
