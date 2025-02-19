package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.MecanumDrive.PARAMS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinMax;
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
import org.firstinspires.ftc.teamcode.commands.PIDMoveCommand;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

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

    public AccelConstraint preloadAccel  = (robotPose, _path, _disp) -> {
        if(_path.length() - _disp < 15) {
            return new MinMax(-30, 30);
        } else {
            return new MinMax(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);
        }
    };


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
        DriveSubsystem drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(7, -61.5, Math.PI/2)), telemetry);

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
                .strafeTo(new Vector2d(2, -29), null, preloadAccel) //-32
                .build();
        Command trajStart = new ActionCommand(trajectoryStart, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectorySweep1 = drive.actionBuilder(new Pose2d (2, -33, Math.PI/2))
                .splineToLinearHeading(new Pose2d(21, -49, Math.toRadians(90)), Math.toRadians(-24.5))
                .splineToLinearHeading(new Pose2d(34.0, -40.0, Math.toRadians(57)), Math.toRadians(56.57))
                .build();
        Command trajSW1 = new ActionCommand(trajectorySweep1, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectorySweep2 = drive.actionBuilder(new Pose2d(27.00, -43.00, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(33.33, -45.16, Math.toRadians(-45)), Math.toRadians(-18.18))
                .build();
        Command trajSW2 = new ActionCommand(trajectorySweep2, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectorySweep3 = drive.actionBuilder(new Pose2d(33.33, -45.16, Math.toRadians(-45)))
                .splineToLinearHeading(new Pose2d(35.00, -43.00, Math.toRadians(45)), Math.toRadians(-25.54))
                .build();
        Command trajSW3 = new ActionCommand(trajectorySweep3, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectorySweep4 = drive.actionBuilder(new Pose2d(35.00, -43.00, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(37.33, -45.16, Math.toRadians(-45)), Math.toRadians(-18.18))
                .build();
        Command trajSW4 = new ActionCommand(trajectorySweep4, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectorySweep5 = drive.actionBuilder(new Pose2d(37.33, -45.16, Math.toRadians(-45)))
                .splineToLinearHeading(new Pose2d(40.00, -43.00, Math.toRadians(45)), Math.toRadians(-25.54))
                .build();
        Command trajSW5 = new ActionCommand(trajectorySweep5, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectorySweep6 = drive.actionBuilder(new Pose2d(40.00, -43.00, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(37, -49.16, Math.toRadians(-45)), Math.toRadians(-18.18))
                .build();
        Command trajSW6 = new ActionCommand(trajectorySweep6, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectorySweep7 = drive.actionBuilder(new Pose2d(37, -49.16, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2)
                .build();
        Command trajSW7 = new ActionCommand(trajectorySweep7, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory2 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .strafeTo(new Vector2d(12, -35))
                .build();
        Command traj2 = new ActionCommand(trajectory2, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory3 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .strafeTo(new Vector2d(10, -35))
                .build();
        Command traj3 = new ActionCommand(trajectory3, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory4 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .strafeTo(new Vector2d(8, -35))
                .build();
        Command traj4 = new ActionCommand(trajectory4, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory5 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .strafeTo(new Vector2d(6, -35))
                .build();
        Command traj5 = new ActionCommand(trajectory5, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory6 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(4, -35))
                .build();
        Command traj6 = new ActionCommand(trajectory6, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome = drive.actionBuilder(new Pose2d (12, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome = new ActionCommand(trajectoryHome, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome1 = drive.actionBuilder(new Pose2d (10, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome1 = new ActionCommand(trajectoryHome1, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome2 = drive.actionBuilder(new Pose2d (8, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome2 = new ActionCommand(trajectoryHome2, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome3 = drive.actionBuilder(new Pose2d (6, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome3 = new ActionCommand(trajectoryHome3, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryEnd = drive.actionBuilder(new Pose2d (4, -35, -Math.PI/2))
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
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.specimen)), // outtake
                new InstantCommand(() -> intakeSlides.manual(-0.3)),
                trajStart,

                new PIDMoveCommand(outtakeSlides, States.OuttakeExtension.post_specimen), // replaces the instant & wait commands
                // new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                // new WaitCommand(300),
                new InstantCommand(outtake::openClaw),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.home)), // intake
                // new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home)),

                new ParallelCommandGroup(
                        trajSW1,
                        new PIDMoveCommand(outtakeSlides, States.OuttakeExtension.home),
                        new SequentialCommandGroup(
                                new WaitCommand(650),
                                new InstantCommand(() -> intakeSlides.manual(0.7)), // to extend
                                new WaitCommand(300),
                                new InstantCommand(() -> intakeSlides.manual(0.2)) // to hold
                        )
                ),
                new InstantCommand(intake::putSweeperDown)

                /*
                trajSW2,
                new InstantCommand(intake::setSweeper)

                 */
                /*
                trajSW3,
                new InstantCommand(intake::putSweeperDown),
                trajSW4,
                new InstantCommand(intake::setSweeper),
                trajSW5,
                new InstantCommand(intake::putSweeperDown),
                trajSW6,
                new InstantCommand(intake::putSweeperUp),
                new InstantCommand(() -> intakeSlides.manual(-0.7)), // to retract
                trajSW7,

                 */

                //new InstantCommand(() -> intakeSlides.manual(-0.3))
                /*, // to hold
                new InstantCommand(() -> outtake.closeClaw()),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new WaitCommand(150),
                traj2,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(500),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home)),
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
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home)),
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
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home)),
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
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home)),
                trajEnd*/

        );
        schedule(auto);
        schedule(new RunCommand(() -> {
            Pose2d pose = drive.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y",pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }));
    }
}
