package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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
@Autonomous(name="SpecimenAuto2", group="Auto")
public class SpecimenAutonomous2 extends CommandOpMode {

    public final MecanumKinematics kinematics = new MecanumKinematics(
            15.984252, 0.8);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(10),
                    new AngularVelConstraint(Math.PI)
            ));
    public final VelConstraint velConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(60),
                    new AngularVelConstraint(Math.PI)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(-70, 70);

    public static double specimenIntakeTime = 0.1;
    public static double specimenOuttakeTime = 0.3;
    public static long endWaitTime = 300;
    public static double startWaitTime = 0.25;
    public static double specimenY = -31;
    public static double specimenY1 = -34;
    public static double specimenY2 = -34;
    public static double specimenY3 = -33;
    public static double specimenX1 = 12;
    public static double specimenX2 = 7;
    public static double specimenX3 = 4;
    public static double specimen1 = -61;
    public static double specimen2 = -61;
    public static double specimen3 = -61;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DriveSubsystem drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(10, -61.5, -Math.PI/2)), telemetry);

        //auto pathing
        Action specimenTrajectoryAction = drive.actionBuilder(drive.getPose())
                .waitSeconds(startWaitTime)
                .strafeTo(new Vector2d(10, -35))
                .waitSeconds(specimenOuttakeTime)
                .strafeTo(new Vector2d(35, -35))
                .strafeTo(new Vector2d(35, -13))
                .splineToLinearHeading(new Pose2d(new Vector2d( 48, -13), Math.PI/2), -Math.PI/2)
                .strafeTo(new Vector2d(48, -48))
                .strafeTo(new Vector2d(48, -13))
                .strafeTo(new Vector2d(57, -13))
                .strafeTo(new Vector2d(57, -48))
                .splineToConstantHeading(new Vector2d(37, -55), -Math.PI/2)
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeToSplineHeading(new Vector2d(13, -35), -Math.PI/2)
                .waitSeconds(specimenOuttakeTime)
                .strafeToSplineHeading(new Vector2d(37, -55), Math.PI/2)
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeToSplineHeading(new Vector2d(7, -35), -Math.PI/2)
                .waitSeconds(specimenOuttakeTime)
                .strafeToSplineHeading(new Vector2d(37, -55), Math.PI/2)
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeToSplineHeading(new Vector2d(4, -35), Math.PI*1.5)
                .build();
        Command specimenTrajectory = new ActionCommand(specimenTrajectoryAction, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryStart = drive.actionBuilder(drive.getPose())
                .waitSeconds(startWaitTime)
                .strafeTo(new Vector2d(10, specimenY))
                .build();
        Command trajStart = new ActionCommand(trajectoryStart, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory1 = drive.actionBuilder(new Pose2d (10, -33, -Math.PI/2))
                .waitSeconds(specimenOuttakeTime)
                .strafeTo(new Vector2d(37, -37))
                .strafeTo(new Vector2d(37, -13))
                .splineToLinearHeading(new Pose2d(new Vector2d( 48, -13), Math.PI/2), -Math.PI/2)
                .strafeTo(new Vector2d(48, -48))
                .strafeTo(new Vector2d(48, -13))
                .strafeTo(new Vector2d(57, -13))
                .strafeTo(new Vector2d(57, -48))
                .strafeTo(new Vector2d(37, -55))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(37, specimen1), defaultVelConstraint)
                .build();
        Command traj1 = new ActionCommand(trajectory1, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory2 = drive.actionBuilder(new Pose2d (37, -60, Math.PI/2))
                .waitSeconds(specimenIntakeTime)
                .strafeToSplineHeading(new Vector2d(specimenX1, specimenY1), -Math.PI/2)
                .build();
        Command traj2 = new ActionCommand(trajectory2, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory3 = drive.actionBuilder(new Pose2d (37, -60, Math.PI/2))
                .waitSeconds(specimenIntakeTime)
                .strafeToSplineHeading(new Vector2d(specimenX2, specimenY2), -Math.PI/2)
                .build();
        Command traj3 = new ActionCommand(trajectory3, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory4 = drive.actionBuilder(new Pose2d (37, -60, Math.PI/2))
                .waitSeconds(specimenIntakeTime)
                .strafeToLinearHeading(new Vector2d(55, -60), -Math.PI/2)
                .build();
        Command traj4 = new ActionCommand(trajectory4, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome = drive.actionBuilder(new Pose2d (13, -35, -Math.PI/2))
                .waitSeconds(specimenOuttakeTime)
                .strafeToSplineHeading(new Vector2d(37, -54), Math.PI/2, velConstraint, defaultAccelConstraint)
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(37, specimen2), defaultVelConstraint)
                .build();
        Command trajHome = new ActionCommand(trajectoryHome, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome1 = drive.actionBuilder(new Pose2d (7, -35, -Math.PI/2))
                .waitSeconds(specimenOuttakeTime)
                .strafeToSplineHeading(new Vector2d(37, -54), Math.PI/2, velConstraint, defaultAccelConstraint)
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(37, specimen3), defaultVelConstraint)
                .build();
        Command trajHome1 = new ActionCommand(trajectoryHome1, Stream.of(drive).collect(Collectors.toSet()));


        Action trajectoryEnd = drive.actionBuilder(drive.getPose())
                .waitSeconds(specimenOuttakeTime)
                .strafeToSplineHeading(new Vector2d(45, -59), Math.PI*.5)
                .build();
        Command trajEnd = new ActionCommand(trajectoryEnd, Stream.of(drive).collect(Collectors.toSet()));

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

        Command auto = new SequentialCommandGroup(
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new InstantCommand(() -> outtake.toggleSpecimenOutput()),
                trajStart,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                traj1,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                traj2,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajHome,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                traj3,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajHome1,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen0)),
                traj4,
                new InstantCommand(() -> outtake.toggleSpecimenOutput()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home))

        );
        schedule(auto);
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
