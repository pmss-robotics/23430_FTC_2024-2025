package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.drive.MecanumDrive.extraCorrection;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.DetectedSample;

import java.util.stream.Collectors;
import java.util.stream.Stream;


public class Methods {
    DetectedSample sample;

    public SequentialCommandGroup autoAlign(VisionSubsystem vision, IntakeSubsystem intake, DriveSubsystem drive, Pose2d start) {
        return new SequentialCommandGroup(
                new InstantCommand(vision::enableDetection),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(vision::disableDetection),
                                new InstantCommand(() -> intake.setWristPosition(sample.angle)),
                                new ActionCommand(
                                        drive.actionBuilder(start)
                                            .strafeTo(start.position.plus(new Vector2d(sample.x, sample.y))).build(), // FIXME x and y might be swapped. since it depends on orientation.
                                        Stream.of(drive).collect(Collectors.toSet())
                                )
                                // FIXME add the sequential routine to intake it here or put it outside idk.
                                // also refactor johnnys repeat code.
                        ),
                        new InstantCommand(vision::disableDetection),
                        () -> {
                            try {sample = vision.getSampleAngle();}
                            catch (Exception e) {return false;}
                            return true;
                        }
                )
        );
    }

    public static InstantCommand enableCorrection() {
        return new InstantCommand(() -> extraCorrection = true);
    }
    public static InstantCommand disableCorrection() {
        return new InstantCommand(() -> extraCorrection = false);
    }

}
