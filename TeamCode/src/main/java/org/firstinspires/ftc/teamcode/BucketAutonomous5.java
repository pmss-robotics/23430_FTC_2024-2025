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
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
@Autonomous(name="BucketAuto5", group="Auto")
public class BucketAutonomous5 extends CommandOpMode {

    final static MecanumKinematics kinematics = new MecanumKinematics(
            15.984252, 0.8);
    final static VelConstraint velConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(60),
                    new AngularVelConstraint(1.141592653589793)
            ));
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(30),
                    new AngularVelConstraint(Math.PI)
            ));

    public static double intakeWaitTime = 2.0;
    public static double outtakeWaitTime = 2.0;
    public static double specimenWaitTime = 0.5;
    public static int intakeWaitMili = 3000;
    public static int outtakeWaitmili = 2000;
    public static int outtakeDropTime = 750;
    public static double sampleAngle1 = 85;
    public static double sampleAngle2 = 109;
    public static double sampleAngle3 = 135;
    public static int samplePos1 = 0;
    public static int samplePos2 = 0;
    public static int samplePos3 = 0;


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DriveSubsystem drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(-40, -61.5, Math.PI/2)), telemetry);

        Action trajectoryAction = drive.actionBuilder(drive.getPose())
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45))
                .waitSeconds(outtakeWaitTime)
                .strafeToLinearHeading(new Vector2d(-48.5, -42), Math.toRadians(90))
                .waitSeconds(intakeWaitTime)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45))
                .waitSeconds(outtakeWaitTime)
                .strafeToLinearHeading(new Vector2d(-58, -42), Math.toRadians(90))
                .waitSeconds(intakeWaitTime)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45))
                .waitSeconds(outtakeWaitTime)
                .strafeToLinearHeading(new Vector2d(-54, -26), Math.toRadians(180))
                .waitSeconds(intakeWaitTime)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45))
                .waitSeconds(outtakeWaitTime)
                .splineToLinearHeading(new Pose2d(-24, -10, Math.PI), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(-35, 60), 0)
                .build();
        Command trajectory = new ActionCommand(trajectoryAction, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory1 = drive.actionBuilder(drive.getPose())
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45), defaultVelConstraint)
                .build();
        Command traj1 = new ActionCommand(trajectory1, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory2 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-48.5, -35), Math.toRadians(90), defaultVelConstraint)
                .build();
        Command traj2 = new ActionCommand(trajectory2, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory3 = drive.actionBuilder(new Pose2d(-48.5, -35, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45), defaultVelConstraint)
                .build();
        Command traj3 = new ActionCommand(trajectory3, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory4 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-58, -35), Math.toRadians(90), defaultVelConstraint)
                .build();
        Command traj4 = new ActionCommand(trajectory4, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory5 = drive.actionBuilder(new Pose2d(-58, -35, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45), defaultVelConstraint)
                .build();
        Command traj5 = new ActionCommand(trajectory5, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory6 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-54, -26), Math.toRadians(180), defaultVelConstraint)
                .build();
        Command traj6 = new ActionCommand(trajectory6, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory7 = drive.actionBuilder(new Pose2d(-54, -26, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45), defaultVelConstraint)
                .build();
        Command traj7 = new ActionCommand(trajectory7, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory8 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-16, -58), Math.toRadians(0))
                .build();
        Command traj8 = new ActionCommand(trajectory8, Stream.of(drive).collect(Collectors.toSet()));

        Action trajectory9 = drive.actionBuilder(new Pose2d(-16, -58, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45), defaultVelConstraint)
                .build();
        Command traj9 = new ActionCommand(trajectory9, Stream.of(drive).collect(Collectors.toSet()));

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
        outtake.setOuttakeState(States.Outtake.preTransfer);
        outtake.closeClaw();
        waitForStart();


        //tune extension position


        Command bucket = new SequentialCommandGroup(
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket)),
                new WaitCommand(outtakeWaitmili),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.bucket)),
                new WaitCommand(outtakeDropTime),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.preTransfer)),
                new WaitCommand(500),
                new InstantCommand(() -> outtakeSlides.toggleBucket())
        );

        Command bucket2 = new SequentialCommandGroup(
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket)),
                new WaitCommand(outtakeWaitmili),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.bucket)),
                new WaitCommand(outtakeDropTime),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.preTransfer)),
                new WaitCommand(500),
                new InstantCommand(() -> outtakeSlides.toggleBucket())
        );

        Command bucket3 = new SequentialCommandGroup(
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket)),
                new WaitCommand(outtakeWaitmili),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.bucket)),
                new WaitCommand(outtakeDropTime),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.preTransfer)),
                new WaitCommand(500),
                new InstantCommand(() -> outtakeSlides.toggleBucket())
        );

        Command bucket4 = new SequentialCommandGroup(
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket)),
                new WaitCommand(outtakeWaitmili),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.bucket)),
                new WaitCommand(outtakeDropTime),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.preTransfer)),
                new WaitCommand(500),
                new InstantCommand(() -> outtakeSlides.toggleBucket())
        );

        Command bucket5 = new SequentialCommandGroup(
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.bucket)),
                new WaitCommand(outtakeWaitmili),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.bucket)),
                new WaitCommand(outtakeDropTime),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.preTransfer)),
                new WaitCommand(500),
                new InstantCommand(() -> outtakeSlides.toggleBucket())
        );

        Command getSample = new SequentialCommandGroup(
                new InstantCommand(() -> intake.openIntakeClaw()),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setIntakeState(States.Intake.intake)),
                new WaitCommand(750),
                new InstantCommand(() -> intake.closeIntakeClaw()),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setIntakeState(States.Intake.middle))
        );

        Command getSample2 = new SequentialCommandGroup(
                new InstantCommand(() -> intake.openIntakeClaw()),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setIntakeState(States.Intake.intake)),
                new WaitCommand(750),
                new InstantCommand(() -> intake.closeIntakeClaw()),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setIntakeState(States.Intake.middle))
        );

        Command getSample3 = new SequentialCommandGroup(
                new InstantCommand(() -> intake.openIntakeClaw()),
                new InstantCommand(() -> intake.rotateLeft()),
                new InstantCommand(() -> intake.rotateLeft()),
                new InstantCommand(() -> intake.rotateLeft()),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setIntakeState(States.Intake.intake)),
                new WaitCommand(750),
                new InstantCommand(() -> intake.closeIntakeClaw()),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setIntakeState(States.Intake.middle))
        );

        Command getSample4 = new SequentialCommandGroup(
                new InstantCommand(() -> intake.openIntakeClaw()),
                new InstantCommand(() -> intake.rotateLeft()),
                new InstantCommand(() -> intake.rotateLeft()),
                new InstantCommand(() -> intake.rotateLeft()),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setIntakeState(States.Intake.intake)),
                new WaitCommand(750),
                new InstantCommand(() -> intake.closeIntakeClaw()),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setIntakeState(States.Intake.middle))
        );

        Command transfer = new SequentialCommandGroup(
                new InstantCommand(() -> intake.setIntakeState(States.Intake.transfer)),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> intakeSlides.manual(-0.7)),
                new WaitCommand(600),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.transfer)),
                new WaitCommand(50),
                new InstantCommand(() -> outtake.closeClaw()),
                new WaitCommand(150),
                new InstantCommand(() -> intake.openIntakeClaw()),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.preTransfer))
        );

        Command transfer2 = new SequentialCommandGroup(
                new InstantCommand(() -> intake.setIntakeState(States.Intake.transfer)),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> intakeSlides.manual(-0.7)),
                new WaitCommand(600),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.transfer)),
                new WaitCommand(50),
                new InstantCommand(() -> outtake.closeClaw()),
                new WaitCommand(150),
                new InstantCommand(() -> intake.openIntakeClaw()),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.preTransfer))
        );

        Command transfer3 = new SequentialCommandGroup(
                new InstantCommand(() -> intake.setIntakeState(States.Intake.transfer)),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> intakeSlides.manual(-0.7)),
                new WaitCommand(600),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.transfer)),
                new WaitCommand(50),
                new InstantCommand(() -> outtake.closeClaw()),
                new WaitCommand(150),
                new InstantCommand(() -> intake.openIntakeClaw()),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.preTransfer))
        );

        Command transfer4 = new SequentialCommandGroup(
                new InstantCommand(() -> intake.setIntakeState(States.Intake.transfer)),
                new InstantCommand(() -> outtake.openClaw()),
                new InstantCommand(() -> intakeSlides.manual(-0.7)),
                new WaitCommand(600),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.transfer)),
                new WaitCommand(50),
                new InstantCommand(() -> outtake.closeClaw()),
                new WaitCommand(150),
                new InstantCommand(() -> intake.openIntakeClaw()),
                new InstantCommand(() -> outtake.setOuttakeState(States.Outtake.preTransfer))
        );


        Command auto = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        traj1,
                        bucket
                ),
                traj2,
                getSample,
                transfer,
                new ParallelCommandGroup(
                        traj3,
                        bucket2
                ),
                traj4,
                getSample2,
                transfer2,
                new ParallelCommandGroup(
                        traj5,
                        bucket3
                ),
                traj6,
                getSample3,
                transfer3,
                new ParallelCommandGroup(
                        traj7,
                        bucket4
                ),
                traj8,
                getSample4,
                transfer4,
                new ParallelCommandGroup(
                        traj9,
                        bucket5
                )

                );

        schedule(auto);
        // TODO: create wrappers for trajectory following maybe possibly
        // this RunCommand Loop might be useless
        schedule(new RunCommand(() -> {
            // TelemetryPacket packet = new TelemetryPacket();
            Pose2d pose = drive.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y",pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
            //packet.fieldOverlay().setStroke("#3F51B5");
            //Drawing.drawRobot(packet.fieldOverlay(), pose);
            //FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }));
    }
}
