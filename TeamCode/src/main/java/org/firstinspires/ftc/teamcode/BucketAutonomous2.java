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
@Autonomous(name="BucketAuto2", group="Auto")
public class BucketAutonomous2 extends CommandOpMode {

    final static MecanumKinematics kinematics = new MecanumKinematics(
            15.984252, 0.8);
    final static VelConstraint velConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(60),
                    new AngularVelConstraint(1.141592653589793)
            ));

    public static double intakeWaitTime = 2.0;
    public static double outtakeWaitTime = 2.0;
    public static double specimenWaitTime = 0.5;
    public static int intakeWaitMili = 3000;
    public static int outtakeWaitmili = 2000;
    public static double sampleAngle1 = 85;
    public static double sampleAngle2 = 109;
    public static double sampleAngle3 = 135;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DriveSubsystem drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(-33, -61.5, Math.PI/2)), telemetry);

        Action trajectoryAction = drive.actionBuilder(drive.getPose())
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), -Math.PI/2)
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(80)), Math.PI)
                .waitSeconds(intakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.PI)
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(105)), Math.PI)
                .waitSeconds(intakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.PI)
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -51, Math.toRadians(110)), Math.PI)
                .waitSeconds(intakeWaitTime)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.PI)
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-33, -61.5, Math.PI/2), -Math.toRadians(45))
//                .strafeToLinearHeading(new Vector2d(-35, 60), 0)
                .build();
        Command trajectory = new ActionCommand(trajectoryAction, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory1 = drive.actionBuilder(drive.getPose())
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(35), velConstraint)
                .build();
        Command traj1 = new ActionCommand(trajectory1, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory2 = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(35)))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(sampleAngle1), velConstraint)
                .build();
        Command traj2 = new ActionCommand(trajectory2, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory3 = drive.actionBuilder(new Pose2d(-52, -52, Math.toRadians(sampleAngle1)))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45), velConstraint)
                .build();
        Command traj3 = new ActionCommand(trajectory3, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory4 = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-52, -50), Math.toRadians(sampleAngle2), velConstraint)
                .build();
        Command traj4 = new ActionCommand(trajectory4, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory5 = drive.actionBuilder(new Pose2d(-52, -50, Math.toRadians(sampleAngle2)))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45), velConstraint)
                .build();
        Command traj5 = new ActionCommand(trajectory5, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory6 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-53, -44), Math.toRadians(sampleAngle3), velConstraint)
                .build();
        Command traj6 = new ActionCommand(trajectory6, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory7 = drive.actionBuilder(new Pose2d(-53, -44, Math.toRadians(sampleAngle3)))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45), velConstraint)
                .build();
        Command traj7 = new ActionCommand(trajectory7, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory8 = drive.actionBuilder(new Pose2d(-53, -50, Math.toRadians(sampleAngle3)))
                .turnTo(Math.toRadians(90))
                .build();
        Command traj8 = new ActionCommand(trajectory8, Stream.of(drive).collect(Collectors.toSet()));

        OuttakeSlidesSubsystem outtakeSlides = new OuttakeSlidesSubsystem(hardwareMap, telemetry);
        outtakeSlides.setDefaultCommand(new RunCommand(outtakeSlides::holdPosition, outtakeSlides));

        IntakeSlidesSubsystem intakeSlides = new IntakeSlidesSubsystem(hardwareMap, telemetry);
        intakeSlides.setDefaultCommand(new RunCommand(intakeSlides::holdPosition, intakeSlides));
/*      try {
            vision = new VisionSubsystem(hardwareMap, telemetry);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
*/
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap, telemetry);

        waitForStart();


        //tune extension position
        Command sample = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intakeSlides.setIntakeSlidesState(States.IntakeExtension.intake)),
                        new InstantCommand(() -> intake.setWristState(States.Intake.intake))
                ),
                new InstantCommand(() -> intake.setPower(1), intake),
                new WaitCommand(800),
                new InstantCommand(() -> intake.setPower(0.5), intake),
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.toggleWristState()),
                        new InstantCommand(() -> intakeSlides.toggleIntakeSlidesState())
                ),
                new InstantCommand(() -> intake.setPower(0), intake),
                new WaitCommand(800),
                new InstantCommand(() -> intake.setPower(0.5), intake)
        );

        Command bucket = new SequentialCommandGroup(
                new InstantCommand(() -> outtake.setWristState(States.Outtake.bucket)),
                new WaitCommand(OuttakeSubsystem.dropTime),
                new InstantCommand(() -> outtake.toggleWristState()),
                new InstantCommand(() -> outtakeSlides.toggleBucket())
        );

        Command intakeSample = new SequentialCommandGroup(
                new InstantCommand(() -> intakeSlides.manual(0.75)),
                new WaitCommand(220),
                new InstantCommand(() -> intakeSlides.manual(0.5)),
                new WaitCommand(100),
                new InstantCommand(() -> intakeSlides.manual(0.4)),
                new WaitCommand(800)

        );
        Command intakeSample2 = new SequentialCommandGroup(
                new InstantCommand(() -> intakeSlides.manual(0.75)),
                new WaitCommand(250),
                new InstantCommand(() -> intakeSlides.manual(0.4)),
                new WaitCommand(750)

        );
        Command intakeSample3 = new SequentialCommandGroup(
                new InstantCommand(() -> intakeSlides.manual(0.7)),
                new WaitCommand(200),
                new InstantCommand(() -> intakeSlides.manual(0.40)),
                new WaitCommand(1500)

        );


        Command auto = new SequentialCommandGroup(
                new InstantCommand(() -> intake.setWristState(States.Intake.transfer)),
                new InstantCommand(() -> outtakeSlides.toggleBucket()),
                traj1,
                new WaitCommand(100),
                bucket,
                new InstantCommand(() -> intake.setWristState(States.Intake.intake)),
                new InstantCommand(() -> intakeSlides.manual(-0.3)),
                traj2,
                new WaitCommand(750),
                new InstantCommand(() -> intake.setPower(1)),
                intakeSample,
                new InstantCommand(() -> intake.setPower(0.5)),
                new InstantCommand(() -> intake.setWristState(States.Intake.home)),
                new InstantCommand(() -> intakeSlides.manual(-0.8)),
                traj3,
                new WaitCommand(450),
                new InstantCommand(() -> intake.setPower(0)),
                new InstantCommand(() -> intakeSlides.manual(-0.3)),
                new WaitCommand(500),
                new InstantCommand(() -> intake.setWristState(States.Intake.transfer)),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket)),
                new InstantCommand(() -> intake.setPower(0.5)),
                new WaitCommand(1500),
                bucket,
                new InstantCommand(() -> intake.setWristState(States.Intake.intake)),
                new InstantCommand(() -> intakeSlides.manual(-0.3)),
                traj4,
                new WaitCommand(750),
                new InstantCommand(() -> intake.setPower(1)),
                intakeSample2,
                new InstantCommand(() -> intake.setPower(0.5)),
                new InstantCommand(() -> intake.setWristState(States.Intake.home)),
                new InstantCommand(() -> intakeSlides.manual(-0.8)),
                traj5,
                new WaitCommand(450),
                new InstantCommand(() -> intake.setPower(0)),
                new InstantCommand(() -> intakeSlides.manual(-0.3)),
                new WaitCommand(500),
                new InstantCommand(() -> intake.setWristState(States.Intake.transfer)),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket)),
                new InstantCommand(() -> intake.setPower(0.5)),
                new WaitCommand(1500),
                bucket,
                new InstantCommand(() -> intake.setWristState(States.Intake.intake)),
                new InstantCommand(() -> intakeSlides.manual(-0.3)),
                traj6,
                new WaitCommand(750),
                new InstantCommand(() -> intake.setPower(1)),
                intakeSample3,
                new InstantCommand(() -> intake.setPower(0.5)),
                new InstantCommand(() -> intake.setWristState(States.Intake.home)),
                new InstantCommand(() -> intakeSlides.manual(-0.8)),
                traj7,
                new WaitCommand(450),
                new InstantCommand(() -> intake.setPower(0)),
                new InstantCommand(() -> intakeSlides.manual(-0.3)),
                new WaitCommand(500),
                new InstantCommand(() -> intake.setWristState(States.Intake.transfer)),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket)),
                new InstantCommand(() -> intake.setPower(0.5)),
                new WaitCommand(1500),
                bucket

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
