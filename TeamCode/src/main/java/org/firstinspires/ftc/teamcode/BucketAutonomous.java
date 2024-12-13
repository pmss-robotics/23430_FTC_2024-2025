package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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

import java.util.Arrays;
import java.util.stream.Collectors;
import java.util.stream.Stream;


@Config
@Autonomous(name="Bucket_Auto", group="Auto")
public class BucketAutonomous extends CommandOpMode {

    final static MecanumKinematics kinematics = new MecanumKinematics(
            15.984252, 0.8);
    final static VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(10),
                    new AngularVelConstraint(Math.PI)
            ));

    public static double intakeWaitTime = 2.0;
    public static double outtakeWaitTime = 2.0;
    public static double specimenWaitTime = 0.5;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DriveSubsystem drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(-10, -61.5,-Math.PI/2)), telemetry);

        Action trajectoryAction = drive.actionBuilder(drive.getPose())
                .strafeTo(new Vector2d(-10, -35))
                .waitSeconds(specimenWaitTime)
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(80)), Math.PI)
                .waitSeconds(intakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.PI)
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(105)), Math.PI)
                .waitSeconds(intakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.PI)
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -51, Math.toRadians(120)), Math.PI)
                .waitSeconds(intakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.PI)
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-24, -10, Math.PI), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(-35, 60), 0)
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
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket)),
                new InstantCommand(() -> outtake.setWristState(States.Outtake.bucket)),
                new WaitCommand(OuttakeSubsystem.dropTime),
                new InstantCommand(() -> outtake.toggleWristState()),
                new InstantCommand(() -> outtakeSlides.toggleBucket())
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


        schedule(trajectory);
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
