package org.firstinspires.ftc.teamcode.util;

public class States {

    // related to each gamepad mapping
    public enum Global {
        home,
        intake_far,
        intake_near,
        bucket,
        specimen,
    }

    // Arm Enums
    public enum IntakeExtension {
        start,
        home,
        intake
    }

    public enum OuttakeExtension {
        start,
        home,
        intake, // is a free value, arm should extend to this but not hold it so driver can adjust
        bucket,
        specimen,
    }

    // Claw Enums
    // Claw merges both wrist and hand states
    public enum Intake {
        start,
        home,
        intake,
        specimen,
        bucket
    }


}