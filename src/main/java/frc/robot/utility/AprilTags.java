package frc.robot.utility;

import edu.wpi.first.apriltag.AprilTag;

public enum AprilTags {
    TAG1(1),
    TAG2(2),
    TAG3(3),
    TAG4(4),
    TAG5(5),
    TAG6(6),
    TAG7(7),
    TAG8(8),
    TAG9(9),
    TAG10(10),
    TAG11(11),
    TAG12(12),
    TAG13(13),
    TAG14(14),
    TAG15(15),
    TAG16(16),
    TAG17(17),
    TAG18(18),
    TAG19(19),
    TAG20(20),
    TAG21(21),
    TAG22(22);

    private int tagID;

    AprilTags(int value) { tagID = value; }
    public int value() { return tagID; }
    public static AprilTags get(int value) {
        switch (value) {
            case 1:
                return TAG1;
            case 2:
                return TAG2;
            case 3:
                return TAG3;
            case 4:
                return TAG4;
            case 5:
                return TAG5;
            case 6:
                return TAG6;
            case 7:
                return TAG7;
            case 8:
                return TAG8;
            case 9:
                return TAG9;
            case 10:
                return TAG10;
            case 11:
                return TAG11;
            case 12:
                return TAG12;
            case 13:
                return TAG13;
            case 14:
                return TAG14;
            case 15:
                return TAG15;
            case 16:
                return TAG16;
            default:
                return null;
        }
     }
}
