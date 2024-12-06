package org.firstinspires.ftc.teamcode.util;

public class States {

    // global states might be nothing
    public enum Global {
        home,
        intake_far,
        intake_near,
        bucket,
        specimen,
    }

    // Intake Enums
    public enum IntakeExtension {
        start,
        home,
        intake,
        middle
    }

    public enum Intake {
        start,
        home,
        intake,
        transfer
    }

    // Outtake Enums
    public enum OuttakeExtension {
        start,
        home,
        bucket,
        specimen0,
        specimen1,
        specimen,
        post_specimen,
        player,
        ascent0,
        ascent
    }

    public enum Outtake {
        start,
        home,
        bucket,
        specimen,
        ascent
    }

    public enum Ascent {
        start,
        home,
        ascent
    }



}