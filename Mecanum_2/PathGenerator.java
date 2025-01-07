package org.firstinspires.ftc.teamcode;



import java.math.BigDecimal;
import java.math.MathContext;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;



public class PathGenerator {
    // NOTE: To visualize splines use this (we use cubic bezier splines as opposed to roadrunner's quintic
    // splines):
    // https://github.com/MrinallU/Cubic-Spline-Interpolator
    // Don't use/edit this unless you know what you are doing...

    // todo: Derive spline trajectory velocity profiles (inches per second for x, y components)
    // todo: Repeat for the theta process
    // resource:
    // https://github.com/acmerobotics/road-runner/blob/master/doc/pdf/Quintic_Splines_for_FTC.pdf
    // resource: helpful python visualizer
    // https://github.com/acmerobotics/road-runner/blob/master/doc/notebook/road-runner-lite.ipynb
    public PathGenerator() {}
    
    
    public static ArrayList<Point> interpSplinePath(ArrayList<Point> pts, Point curLoc) {
        ArrayList<Point> wp = new ArrayList<>();
        ArrayList<Double> slopes = new ArrayList<>();
        pts.add(0, curLoc); // add current point to the front

        for (int i = 0; i < pts.size() - 1; i++) {
            if (pts.get(i + 1).xP == pts.get(i).xP) {
                slopes.add(-1.0);
            } else {
                slopes.add(((pts.get(i + 1).yP - pts.get(i).yP) / (pts.get(i).xP - pts.get(i + 1).xP)));
            }
        }

        if (pts.get(pts.size() - 1).xP == pts.get(pts.size() - 2).xP) {
            slopes.add(-1.0);
        } else {
            slopes.add((pts.get(pts.size() - 1).yP - pts.get(pts.size() - 2).yP) / (pts.get(pts.size() - 2).xP - pts.get(pts.size() - 1).xP));
        }

        Point prev = new Point(Double.MAX_VALUE, Double.MAX_VALUE);

        for (int i = 0; i < pts.size() - 1; i++) {
            Point p1, p2;
            p1 = pts.get(i);
            p2 = pts.get(i + 1);

            if (p1.xP == p2.xP) {
                double inc = (p2.yP - p1.yP) / 40;
                if (p1.yP <= p2.yP) {
                    for (double j = p1.yP; j <= p2.yP; j += inc) {
                        wp.add(new Point(p1.xP, j));
                    }
                } else {
                    for (double j = p1.yP; j >= p2.yP; j += inc) {
                        wp.add(new Point(p1.xP, j));
                    }
                }
            } else {
                double slope1, slope2, x1, x2;
                if (prev.xP == Double.MAX_VALUE) {
                    slope1 = slopes.get(i);
                } else {
                    slope1 = (pts.get(i).yP - prev.yP) / (prev.xP - pts.get(i).xP);
                }

                slope2 = slopes.get(i + 1);
                if (i + 2 < pts.size()) {
                    boolean pos1 = (pts.get(i + 1).xP - pts.get(i).xP) >= 0;
                    boolean pos2 = (pts.get(i + 2).xP - pts.get(i + 1).xP) >= 0;

                    if (pos2 != pos1) {
                        slope2 = -slopes.get(i + 1);
                    }
                }

                if (prev.xP == Double.MAX_VALUE) {
                    x1 = ((p2.xP - p1.xP) / 4) + p1.xP;
                } else {
                    double mult;
                    if (p1.xP - prev.xP > 0) {
                        mult = 1;
                    } else {
                        mult = -1;
                    }
                    x1 = p1.xP + ((Math.abs(p2.xP - p1.xP) / 4) * mult);
                }

                x2 = (3 * ((p2.xP - p1.xP) / 4)) + p1.xP;
                Point c0, c1, c2, c3;
                c0 = new Point(p1.xP, p1.yP);
                c1 = new Point(x1, (slope1 * (p1.xP - x1) + p1.yP));
                c2 = new Point(x2, (slope2 * (p2.xP - x2) + p2.yP));
                c3 = new Point(p2.xP, p2.yP);
                prev = c2;

                // increase increment for less weigh points!
                for (double t = 0; t <= 1; t += 0.025) {
                    wp.add(new Point((1 - t) * ((1 - t) * ((1 - t) * c0.xP + t * c1.xP) + t * ((1 - t) * c1.xP + t * c2.xP)) + t * ((1 - t) * ((1 - t) * c1.xP + t * c2.xP) + t * ((1 - t) * c2.xP + t * c3.xP)),
                            ((1 - t) * ((1 - t) * ((1 - t) * c0.yP + t * c1.yP) + t * ((1 - t) * c1.yP + t * c2.yP)) + t * ((1 - t) * ((1 - t) * c1.yP + t * c2.yP) + t * ((1 - t) * c2.yP + t * c3.yP))),  p2.ang, p2.spline, p2.invertSpline, p2.speed));
                }
            }
        }
        return wp;
    }


}
