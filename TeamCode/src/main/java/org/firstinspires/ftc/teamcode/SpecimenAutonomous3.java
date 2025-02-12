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
@Autonomous(name="SpecimenAuto3", group="Auto")
public class SpecimenAutonomous3 extends CommandOpMode {

    public final MecanumKinematics kinematics = new MecanumKinematics(
            15.984252, 0.8);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(30),
                    new AngularVelConstraint(Math.PI)
            ));
    public final VelConstraint secondVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(15),
                    new AngularVelConstraint(Math.PI)
            ));
    public final VelConstraint velConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(60),
                    new AngularVelConstraint(Math.PI)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(-70, 70);

    public static double specimenIntakeTime = 0.2;
    public static double specimenOuttakeTime = 0.5;
    public static long endWaitTime = 300;
    public static double startWaitTime = 0.35;
    public static double specimenY = -31;
    public static double specimenY1 = -34;
    public static double specimenY2 = -34;
    public static double specimenY3 = -33;
    public static double specimenX1 = 12;
    public static double specimenX2 = 10;
    public static double specimenX3 = 7;
    public static double specimen1 = -64.5;
    public static double specimen2 = -62;
    public static double specimen3 = -63;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DriveSubsystem drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(4, -61.5, Math.PI/2)), telemetry);

        //auto pathing
        Action specimenTrajectoryAction = drive.actionBuilder(drive.getPose())
                .waitSeconds(startWaitTime)
                .strafeTo(new Vector2d(10, -35))
                .waitSeconds(specimenOuttakeTime)
                .strafeTo(new Vector2d(35, -35))
                .strafeTo(new Vector2d(35, -13))
                .splineToConstantHeading(new Vector2d( 48, -13), -Math.PI/2)
                .strafeTo(new Vector2d(48, -48))
                .strafeTo(new Vector2d(48, -13))
                .splineToConstantHeading(new Vector2d( 57, -13), -Math.PI/2)
                .strafeTo(new Vector2d(57, -48))
                .strafeTo(new Vector2d(57, -13))
                .splineToConstantHeading(new Vector2d( 62, -13), -Math.PI/2)
                .strafeTo(new Vector2d(62, -48))
                .splineToConstantHeading(new Vector2d(37, -55), -Math.PI/2)
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(13, -35))
                .waitSeconds(specimenOuttakeTime)
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(7, -35))
                .waitSeconds(specimenOuttakeTime)
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(4, -35))
                .waitSeconds(specimenOuttakeTime)
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(1, -35))
                .waitSeconds(specimenOuttakeTime)

                .build();
        Command specimenTrajectory = new ActionCommand(specimenTrajectoryAction, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryStart = drive.actionBuilder(drive.getPose())
                .waitSeconds(startWaitTime)
                .strafeTo(new Vector2d(10, -35))
                .build();
        Command trajStart = new ActionCommand(trajectoryStart, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory1 = drive.actionBuilder(new Pose2d (4, -33, -Math.PI/2))
                .strafeTo(new Vector2d(35, -35))
                .strafeTo(new Vector2d(35, -13))
                .splineToConstantHeading(new Vector2d( 48, -13), -Math.PI/2)
                .strafeTo(new Vector2d(48, -48))
                .strafeTo(new Vector2d(48, -13))
                .splineToConstantHeading(new Vector2d( 57, -13), -Math.PI/2)
                .strafeTo(new Vector2d(57, -48))
                .strafeTo(new Vector2d(57, -13))
                .splineToConstantHeading(new Vector2d( 62, -13), -Math.PI/2)
                .strafeTo(new Vector2d(62, -48))
                .splineToConstantHeading(new Vector2d(37, -55), Math.PI)
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command traj1 = new ActionCommand(trajectory1, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory2 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(13, -35))
                .build();
        Command traj2 = new ActionCommand(trajectory2, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory3 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(7, -35))
                .build();
        Command traj3 = new ActionCommand(trajectory3, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory4 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(4, -35))
                .build();
        Command traj4 = new ActionCommand(trajectory4, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory5 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(1, -35))
                .build();
        Command traj5 = new ActionCommand(trajectory5, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome = drive.actionBuilder(new Pose2d (13, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome = new ActionCommand(trajectoryHome, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome1 = drive.actionBuilder(new Pose2d (7, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome1 = new ActionCommand(trajectoryHome1, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome2 = drive.actionBuilder(new Pose2d (4, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome2 = new ActionCommand(trajectoryHome2, Stream.of(drive).collect(Collectors.toSet()));



        Action trajectoryEnd = drive.actionBuilder(new Pose2d (1, -35, -Math.PI/2))
                .strafeToConstantHeading(new Vector2d(45, -59))
                .build();
        Command trajEnd = new ActionCommand(trajectoryEnd, Stream.of(drive).collect(Collectors.toSet()));

        OuttakeSlidesSubsystem outtakeSlides = new OuttakeSlidesSubsystem(hardwareMap, telemetry);
        outtakeSlides.setDefaultCommand(new RunCommand(outtakeSlides::holdPosition, outtakeSlides));
        IntakeSlidesSubsystem intakeSlides = new IntakeSlidesSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap, telemetry);
        TeleOp.currentMode = States.Mode.specimen;

        waitForStart();

        //specimen cycle system

        Command auto = new SequentialCommandGroup(
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                trajStart,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(500),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                traj1,
                new InstantCommand(() -> outtake.closeClaw()),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new WaitCommand(150),
                traj2,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(500),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajHome,
                new InstantCommand(() -> outtake.closeClaw()),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new WaitCommand(150),
                traj3,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(500),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajHome1,
                new InstantCommand(() -> outtake.closeClaw()),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new WaitCommand(150),
                traj4,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(500),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajHome2,
                new InstantCommand(() -> outtake.closeClaw()),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new WaitCommand(150),
                traj5,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(500),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajEnd,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home))

        );
        schedule(specimenTrajectory);
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
