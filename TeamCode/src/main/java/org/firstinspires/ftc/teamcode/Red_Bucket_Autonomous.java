package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
public class Red_Bucket_Autonomous extends CommandOpMode {
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DriveSubsystem drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(-38, -61.5,Math.PI/2)), telemetry);

        Action trajectoryAction = drive.actionBuilder(drive.getPose())
                .lineToYLinearHeading(-35, Math.PI*1.5)
                .waitSeconds(3)
/*                .strafeToLinearHeading(new Vector2d(-58, -45), -Math.PI/2)
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-54.5, -54.5), 5*Math.PI/4)
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-52, -45), Math.toRadians(255))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-54.5, -54.5), 5*Math.PI/4)
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-34, -10), Math.PI) */
//                .strafeToLinearHeading(new Vector2d(35, -60), Math.PI)
                .build();
        Command trajectory = new ActionCommand(trajectoryAction, Stream.of(drive).collect(Collectors.toSet()));

        OuttakeSlidesSubsystem outtakeSlides = new OuttakeSlidesSubsystem(hardwareMap, telemetry);
        outtakeSlides.setDefaultCommand(new RunCommand(outtakeSlides::holdPosition, outtakeSlides));

        IntakeSlidesSubsystem intakeSlides = new IntakeSlidesSubsystem(hardwareMap, telemetry);
/*      try {
            vision = new VisionSubsystem(hardwareMap, telemetry);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
*/
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap, telemetry);

        Command bucket = new SequentialCommandGroup(
                new InstantCommand(() -> intake.setPosition(60), intake),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket), outtakeSlides),
                new InstantCommand(() -> outtake.setWristState(States.Outtake.bucket), outtake),
                new WaitCommand(OuttakeSubsystem.dropTime),
                new InstantCommand(() -> outtake.toggleWristState(), outtake),
                new InstantCommand(() -> outtakeSlides.toggleBucket(), outtakeSlides)
        );

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

        Command auto = new ParallelCommandGroup(
                new SequentialCommandGroup(new WaitCommand(1050), bucket)
/*                new ParallelCommandGroup(
                        new SequentialCommandGroup(new WaitCommand(5050), sample),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(new WaitCommand(7700), bucket),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(new WaitCommand(11600), sample),
                                        new SequentialCommandGroup(new WaitCommand(14300), bucket)
                                        //TODO level 1 ascent
                                )
                        )
                )*/
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
