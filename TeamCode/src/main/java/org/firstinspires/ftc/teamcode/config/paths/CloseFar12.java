package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

public class CloseFar12 {
    private final Follower f;

    public Pose start = new Pose(24-3.5, 120+2.5, Math.toRadians(90));
    public Pose scoreEnd = new Pose(48, 96, Math.toRadians(135));
    public Pose line2Start = new Pose(20, 84, Math.toRadians(180));
    public Pose line2End = new Pose(48.750, 85.250, Math.toRadians(-135));
    public Pose line3End = new Pose(16, 60.050, Math.toRadians(-170));
    public Pose line4Start = new Pose(16, 60.050, Math.toRadians(-170));
    public Pose line4End = new Pose(48.750, 85.250, Math.toRadians(-135));
    public Pose line5End = new Pose(16.250, 70.000, Math.toRadians(0));
    public Pose line6End = new Pose(16, 39.750, Math.toRadians(180));
    public Pose line7End = new Pose(59.500, 10.000, Math.toRadians(180));
    public Pose line8End = new Pose(12.000, 10.000, Math.toRadians(180));
    public Pose line9End = new Pose(59.500, 12.000, Math.toRadians(180));
    public Pose line10End = new Pose(12.000, 12.000, Math.toRadians(180));
    public Pose line11End = new Pose(59.500, 14.000, Math.toRadians(180));
    public Pose line12End = new Pose(12.000, 14.000, Math.toRadians(180));
    public Pose line13End = new Pose(59.500, 16.000, Math.toRadians(180));
    public Pose line14End = new Pose(12.000, 16.000, Math.toRadians(180));

    private int index = 0;
    private static final int PATH_COUNT = 14;

    public CloseFar12(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.RED)) {
            start = start.mirror();
            scoreEnd = scoreEnd.mirror();
            line2Start = line2Start.mirror();
            line2End = line2End.mirror();
            line3End = line3End.mirror();
            line4Start = line4Start.mirror();
            line4End = line4End.mirror();
            line5End = line5End.mirror();
            line6End = line6End.mirror();
            line7End = line7End.mirror();
            line8End = line8End.mirror();
            line9End = line9End.mirror();
            line10End = line10End.mirror();
            line11End = line11End.mirror();
            line12End = line12End.mirror();
            line13End = line13End.mirror();
            line14End = line14End.mirror();
        }

        index = 0;
    }

    public PathChain score() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                start,
                                new Pose(55.593, 94.779),
                                scoreEnd
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), scoreEnd.getHeading())
                .build();
    }

    public PathChain line1() { // intake 1
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scoreEnd,
                                new Pose(50.000, 80.000),
                                line2Start
                        )
                )
                .setLinearHeadingInterpolation(scoreEnd.getHeading(), line2Start.getHeading(), 0.3)
                .build();
    }

    public PathChain line2() { // go back from intake 1 to score
        return f.pathBuilder()
                .addPath(new BezierLine(line2Start, line2End))
                .setLinearHeadingInterpolation(line2Start.getHeading(), line2End.getHeading())
                .build();
    }

    public PathChain line3() { // intake 2
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                line2End,
                                new Pose(55.400, 66.300),
                                line3End
                        )
                )
                .setLinearHeadingInterpolation(line2End.getHeading(), line3End.getHeading())
                .build();
    }

    public PathChain line4() { // go score intake 2
        return f.pathBuilder()
                .addPath(new BezierLine(line4Start, line4End))
                .setLinearHeadingInterpolation(line4Start.getHeading(), line4End.getHeading())
                .setReversed()
                .build();
    }

    public PathChain line5() { // do the gate
        return f.pathBuilder()
                .addPath(new BezierLine(line4End, line5End))
                .setLinearHeadingInterpolation(line4End.getHeading(), line5End.getHeading(), 0.5)
                .build();
    }

    public PathChain line6() { // intake 3
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                line5End,
                                new Pose(72, 44.000),
                                line6End
                        )
                )
                .setLinearHeadingInterpolation(line5End.getHeading(), line6End.getHeading())
                .build();
    }

    public PathChain line7() { // score intake 3
        return f.pathBuilder()
                .addPath(new BezierLine(line6End, line7End))
                .setLinearHeadingInterpolation(line6End.getHeading(), line7End.getHeading())
                .build();
    }

    public PathChain line8() { // to corner
        return f.pathBuilder()
                .addPath(new BezierLine(line7End, line8End))
                .setConstantHeadingInterpolation(line7End.getHeading())
                .build();
    }

    public PathChain line9() { // score corner
        return f.pathBuilder()
                .addPath(new BezierLine(line8End, line9End))
                .setConstantHeadingInterpolation(line8End.getHeading())
                .build();
    }

    public PathChain line10() { // to corner
        return f.pathBuilder()
                .addPath(new BezierLine(line9End, line10End))
                .setConstantHeadingInterpolation(line9End.getHeading())
                .build();
    }

    public PathChain line11() { // score corner
        return f.pathBuilder()
                .addPath(new BezierLine(line10End, line11End))
                .setConstantHeadingInterpolation(line10End.getHeading())
                .build();
    }

    public PathChain line12() { // to corner
        return f.pathBuilder()
                .addPath(new BezierLine(line11End, line12End))
                .setConstantHeadingInterpolation(line11End.getHeading())
                .build();
    }

    public PathChain line13() { // score corner
        return f.pathBuilder()
                .addPath(new BezierLine(line12End, line13End))
                .setConstantHeadingInterpolation(line12End.getHeading())
                .build();
    }

    public PathChain line14() { // intake corner
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                line13End,
                                new Pose(29.469, 16.885),
                                line14End
                        )
                )
                .setConstantHeadingInterpolation(line13End.getHeading())
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return score();
            case 1: return line1();
            case 2: return line2();
            case 3: return line3();
            case 4: return line4();
            case 5: return line5();
            case 6: return line6();
            case 7: return line7();
            case 8: return line8();
            case 9: return line9();
            case 10: return line10();
            case 11: return line11();
            case 12: return line12();
            case 13: return line13();
            case 14: return line14();
            default: return null;
        }
    }

    public boolean hasNext() {
        return index < PATH_COUNT;
    }

    public void reset() {
        index = 0;
    }
}
