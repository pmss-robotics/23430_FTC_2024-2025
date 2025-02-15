package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
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
public class SpecimenAutoTest extends CommandOpMode {

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





    public static double specimenIntakeTime = 0.2;
    public static double specimenOuttakeTime = 0.5;
    public static long endWaitTime = 300;
    public static double startWaitTime = 0.1;
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
    public static int intakeTime = 100;
    public static int outtakeTime = 100;

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
                .strafeTo(new Vector2d(-2, -35))
                .build();
        Command trajStart = new ActionCommand(trajectoryStart, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory1 = drive.actionBuilder(new Pose2d (4, -33, -Math.PI/2))
                .splineToLinearHeading(new Pose2d(27.00, -43.00, Math.toRadians(45)), Math.toRadians(-25.54))
                .splineToLinearHeading(new Pose2d(33.33, -45.16, Math.toRadians(-45)), Math.toRadians(-18.18))
                .splineToLinearHeading(new Pose2d(35.00, -43.00, Math.toRadians(45)), Math.toRadians(-25.54))
                .splineToLinearHeading(new Pose2d(37.33, -45.16, Math.toRadians(-45)), Math.toRadians(-18.18))
                .splineToLinearHeading(new Pose2d(40.00, -43.00, Math.toRadians(45)), Math.toRadians(-25.54))
                .splineToLinearHeading(new Pose2d(30, -45.16, Math.toRadians(-45)), Math.toRadians(-18.18))
                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2)
                .build();
        Command traj1 = new ActionCommand(trajectory1, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory2 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .strafeTo(new Vector2d(13, -35))
                .build();
        Command traj2 = new ActionCommand(trajectory2, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory3 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .strafeTo(new Vector2d(10, -35))
                .build();
        Command traj3 = new ActionCommand(trajectory3, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory4 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .strafeTo(new Vector2d(7, -35))
                .build();
        Command traj4 = new ActionCommand(trajectory4, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory5 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .strafeTo(new Vector2d(4, -35))
                .build();
        Command traj5 = new ActionCommand(trajectory5, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory6 = drive.actionBuilder(new Pose2d (37, -60, -Math.PI/2))
                .waitSeconds(specimenIntakeTime)
                .strafeTo(new Vector2d(1, -35))
                .build();
        Command traj6 = new ActionCommand(trajectory6, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome = drive.actionBuilder(new Pose2d (13, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome = new ActionCommand(trajectoryHome, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome1 = drive.actionBuilder(new Pose2d (10, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome1 = new ActionCommand(trajectoryHome1, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome2 = drive.actionBuilder(new Pose2d (7, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome2 = new ActionCommand(trajectoryHome2, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryHome3 = drive.actionBuilder(new Pose2d (7, -35, -Math.PI/2))
                .strafeTo(new Vector2d(37, -55))
                .strafeTo(new Vector2d(37, -60), defaultVelConstraint)
                .build();
        Command trajHome3 = new ActionCommand(trajectoryHome3, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectoryEnd = drive.actionBuilder(new Pose2d (1, -35, -Math.PI/2))
                .strafeToConstantHeading(new Vector2d(45, -59))
                .build();
        Command trajEnd = new ActionCommand(trajectoryEnd, Stream.of(drive).collect(Collectors.toSet()));

        OuttakeSlidesSubsystem outtakeSlides = new OuttakeSlidesSubsystem(hardwareMap, telemetry);
        outtakeSlides.setDefaultCommand(new RunCommand(outtakeSlides::holdPosition, outtakeSlides));
        IntakeSlidesSubsystem intakeSlides = new IntakeSlidesSubsystem(hardwareMap, telemetry);
        intakeSlides.setDefaultCommand(new RunCommand(intakeSlides::holdPosition, intakeSlides));
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap, telemetry);
        TeleOp.currentMode = States.Mode.specimen;

        GamepadEx driver1 = new GamepadEx(gamepad1);

        boolean input = true;
        boolean getSample = false;
        int numExtension = 0;
        double numSide = 0;
        int numRotation = 0;

        while (input) {
            if (driver1.getButton(GamepadKeys.Button.X)) {
                getSample = true;
            }
            if (driver1.getButton(GamepadKeys.Button.Y)) {
                numExtension++;
            }
            if (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                numRotation++;
            }
            if (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                numRotation--;
            }
            if (driver1.getButton(GamepadKeys.Button.A)) {
                numSide++;
            }
            if (driver1.getButton(GamepadKeys.Button.B)) {
                numSide--;
            }
            if (getSample) {
                telemetry.addData("Sample from Sub: ", "true");
                telemetry.addData("autoExtension", numExtension);
                telemetry.addData("autoMove", numSide);
                telemetry.addData("autoRotation", numRotation);
            } else {
                telemetry.addData("Sample from Sub: ", "false");
            }
            if (driver1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                input = false;
            }
        }
        final int numEx = numExtension*24;
        final int numRot = numRotation*25;
        Action trajectorySample = drive.actionBuilder(new Pose2d(-2, -33, Math.PI/2))
                .strafeTo(new Vector2d(4+numSide*1.5, -33))
                .build();
        Command trajS = new ActionCommand(trajectorySample, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectorySample1 = drive.actionBuilder(new Pose2d(4+numSide*1.5, -33, Math.PI/2))
                .strafeToLinearHeading(new Vector2d(25, -45), -Math.PI/4)
                .build();
        Command trajS1 = new ActionCommand(trajectorySample1, Stream.of(drive).collect(Collectors.toSet()));


        waitForStart();
        Command auto;
        //specimen cycle system
        if (getSample) {
            auto = new SequentialCommandGroup(
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                    new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.specimen)),
                    trajStart,
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                    new InstantCommand(() -> intakeSlides.intakePosition(numEx)),
                    new InstantCommand(() -> intake.openIntakeClaw()),
                    new InstantCommand(() -> intake.setIntakeState(States.Intake.middle)),
                    new InstantCommand(() -> intake.rotate(numRot)),
                    new WaitCommand(outtakeTime),
                    new InstantCommand(() -> outtake.openClaw()),
                    trajS,
                    new InstantCommand(() -> intake.toggleIntakeState()),
                    new WaitCommand(350),
                    new InstantCommand(() -> intake.closeIntakeClaw()),
                    new WaitCommand(100),
                    new InstantCommand(() -> intake.setIntakeState(States.Intake.middle)),
                    new InstantCommand(() -> intake.rotateCenter()),
                    new WaitCommand(250),
                    trajS1,
                    new InstantCommand(() -> intake.openIntakeClaw())/*,
                traj1,
                new InstantCommand(() -> outtake.closeClaw()),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new WaitCommand(intakeTime),
                traj2,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(outtakeTime),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajHome,
                new InstantCommand(() -> outtake.closeClaw()),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new WaitCommand(intakeTime),
                traj3,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(outtakeTime),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajHome1,
                new InstantCommand(() -> outtake.closeClaw()),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new WaitCommand(intakeTime),
                traj4,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(outtakeTime),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajHome2,
                new InstantCommand(() -> outtake.closeClaw()),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new WaitCommand(intakeTime),
                traj5,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(outtakeTime),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajHome3,
                new InstantCommand(() -> outtake.closeClaw()),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                new WaitCommand(intakeTime),
                traj6,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                new WaitCommand(outtakeTime),
                new InstantCommand(() -> outtake.toggleOuttakeState()),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                trajEnd,
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home))*/
            );
        } else {
            auto = new SequentialCommandGroup(
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                    new InstantCommand(() -> outtake.toggleOuttakeState()),
                    trajStart,
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                    new WaitCommand(outtakeTime),
                    new InstantCommand(() -> outtake.toggleOuttakeState()),
                    new InstantCommand(() -> outtake.openClaw()),
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                    traj1,
                    new InstantCommand(() -> outtake.closeClaw()),
                    new InstantCommand(() -> outtake.toggleOuttakeState()),
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                    new WaitCommand(intakeTime),
                    traj2,
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                    new WaitCommand(outtakeTime),
                    new InstantCommand(() -> outtake.toggleOuttakeState()),
                    new InstantCommand(() -> outtake.openClaw()),
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                    trajHome,
                    new InstantCommand(() -> outtake.closeClaw()),
                    new InstantCommand(() -> outtake.toggleOuttakeState()),
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                    new WaitCommand(intakeTime),
                    traj3,
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                    new WaitCommand(outtakeTime),
                    new InstantCommand(() -> outtake.toggleOuttakeState()),
                    new InstantCommand(() -> outtake.openClaw()),
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                    trajHome1,
                    new InstantCommand(() -> outtake.closeClaw()),
                    new InstantCommand(() -> outtake.toggleOuttakeState()),
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                    new WaitCommand(intakeTime),
                    traj4,
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                    new WaitCommand(outtakeTime),
                    new InstantCommand(() -> outtake.toggleOuttakeState()),
                    new InstantCommand(() -> outtake.openClaw()),
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                    trajHome2,
                    new InstantCommand(() -> outtake.closeClaw()),
                    new InstantCommand(() -> outtake.toggleOuttakeState()),
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.specimen)),
                    new WaitCommand(intakeTime),
                    traj5,
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.post_specimen)),
                    new WaitCommand(outtakeTime),
                    new InstantCommand(() -> outtake.toggleOuttakeState()),
                    new InstantCommand(() -> outtake.openClaw()),
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.player)),
                    trajEnd,
                    new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home))

            );
        }
        schedule(auto);
        // TODO: create wrappers for trajectory following maybe possibly
        // this RunCommand Loop might be useless
        schedule(new RunCommand(() -> {
            Pose2d pose = drive.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y",pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }));
    }
}
