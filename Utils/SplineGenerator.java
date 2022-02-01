package org.firstinspires.ftc.teamcode.Utils;

import java.math.BigDecimal;
import java.math.MathContext;
import java.util.ArrayList;
import java.util.Arrays;

public class SplineGenerator {


    // NOTE: To visualize splines use this (we use cubic splines as opposed to roadrunner's quintic splines):
    // https://github.com/MrinallU/Cubic-Spline-Interpolator
    // Don't use/edit this unless you know what you are doing...

    // todo: Derive spline trajectory velocity profiles (inches per second for x, y components)
    // todo: Repeat for the theta process
    // resource: https://github.com/acmerobotics/road-runner/blob/master/doc/pdf/Quintic_Splines_for_FTC.pdf
    // resource: helpful python visualizer https://github.com/acmerobotics/road-runner/blob/master/doc/notebook/road-runner-lite.ipynb
    public SplineGenerator(){

    }

    /**
     *
     * @param p Array of points which the spline will interpolate
     * @return A series of waypoints which a differential drive robot can follow.
     */
    public ArrayList<Point> generateSplinePath(Point [] p, int step) {
        int row = 0;
        int solutionIndex = (p.length - 1) * 4;
        Arrays.sort(p);
        // initialize matrix
        BigDecimal[][] m = new BigDecimal[(p.length - 1) * 4][(p.length - 1) * 4 + 1]; // rows
        for (int  i = 0; i < (p.length - 1) * 4; i++) {
            for (int j = 0; j <= (p.length - 1) * 4; j++) {
                m[i][j] = BigDecimal.ZERO; // fill with zeros
            }
        }

        // n - 1 splines
        for (int functionNr = 0; functionNr < p.length - 1; functionNr++, row++) {
            Point p0 = p[functionNr], p1 = p[functionNr + 1];
            m[row][functionNr * 4] = new BigDecimal(p0.xP, MathContext.DECIMAL64).pow(3, MathContext.DECIMAL64);
            m[row][functionNr * 4 + 1] = new BigDecimal(p0.xP, MathContext.DECIMAL64).pow(2, MathContext.DECIMAL64);
            m[row][functionNr * 4 + 2] = new BigDecimal(p0.xP, MathContext.DECIMAL64);
            m[row][functionNr * 4 + 3] = new BigDecimal(1, MathContext.DECIMAL64);
            m[row][solutionIndex] = new BigDecimal(p0.yP, MathContext.DECIMAL64);

            ++row;

            m[row][functionNr * 4] = new BigDecimal(p1.xP, MathContext.DECIMAL64).pow(3, MathContext.DECIMAL64);
            m[row][functionNr * 4 + 1] = new BigDecimal(p1.xP, MathContext.DECIMAL64).pow(2, MathContext.DECIMAL64);
            m[row][functionNr * 4 + 2] = new BigDecimal(p1.xP, MathContext.DECIMAL64);
            m[row][functionNr * 4 + 3] = new BigDecimal(1, MathContext.DECIMAL64);
            m[row][solutionIndex] = new BigDecimal(p1.yP, MathContext.DECIMAL64);

        }

        // first derivative
        for (int functionNr = 0; functionNr < p.length - 2; functionNr++, row++) {
            Point p1 = p[functionNr + 1];
            m[row][functionNr * 4] = new BigDecimal(3, MathContext.DECIMAL64).multiply(new BigDecimal(p1.xP).pow(2, MathContext.DECIMAL64));
            m[row][functionNr * 4 + 1] = new BigDecimal(2, MathContext.DECIMAL64).multiply(new BigDecimal(p1.xP), MathContext.DECIMAL64);
            m[row][functionNr * 4 + 2] = new BigDecimal(1, MathContext.DECIMAL64);
            m[row][functionNr * 4 + 4] = new BigDecimal(-3).multiply(new BigDecimal(p1.xP).pow(2, MathContext.DECIMAL64));
            m[row][functionNr * 4 + 5] = new BigDecimal(-2, MathContext.DECIMAL64).multiply(new BigDecimal(p1.xP), MathContext.DECIMAL64);
            m[row][functionNr * 4 + 6] = new BigDecimal(-1, MathContext.DECIMAL64);

        }


        // second derivative
        for (int functionNr = 0; functionNr < p.length - 2; functionNr++, row++) {
            Point p1 = p[functionNr + 1];
            m[row][functionNr * 4] = new BigDecimal(6, MathContext.DECIMAL64).multiply(new BigDecimal(p1.xP, MathContext.DECIMAL64), MathContext.DECIMAL64);
            m[row][functionNr * 4 + 1] = new BigDecimal(2, MathContext.DECIMAL64);
            m[row][functionNr * 4 + 4] = new BigDecimal(-6, MathContext.DECIMAL64).multiply(new BigDecimal(p1.xP, MathContext.DECIMAL64), MathContext.DECIMAL64);
            m[row][functionNr * 4 + 5] = new BigDecimal(-2, MathContext.DECIMAL64);
        }


        // check these calculations later
        m[row][0] = new BigDecimal(6, MathContext.DECIMAL64).multiply(new BigDecimal(p[0].xP, MathContext.DECIMAL64), MathContext.DECIMAL64);
        m[row++][1] = new BigDecimal(2, MathContext.DECIMAL64);
        m[row][solutionIndex - 4] = new BigDecimal(6, MathContext.DECIMAL64).multiply(new BigDecimal(p[p.length - 1].xP, MathContext.DECIMAL64), MathContext.DECIMAL64);
        m[row][solutionIndex - 4 + 1] = new BigDecimal(2, MathContext.DECIMAL64);


        BigDecimal[][] reducedRowEchelonForm = rref(m);
        BigDecimal[] coefficients = new BigDecimal[reducedRowEchelonForm.length];
        for (int i = 0; i < reducedRowEchelonForm.length; i++) {
            coefficients[i] = reducedRowEchelonForm[i][reducedRowEchelonForm[i].length - 1];
        }

        ArrayList<Point> path = new ArrayList<>();
        for (int i = 0; i < coefficients.length; i += 4) {
            for (double j = p[i / 4].xP; j <= p[(i / 4) + 1].xP; j += step) {
                BigDecimal a = coefficients[i].multiply(BigDecimal.valueOf(j).pow(3, MathContext.DECIMAL64));
                BigDecimal b = coefficients[i + 1].multiply(BigDecimal.valueOf(j).pow(2, MathContext.DECIMAL64));
                BigDecimal c = coefficients[i + 2].multiply(BigDecimal.valueOf(j));
                BigDecimal d = coefficients[i + 3];
                path.add(new Point(j, a.add(b).add(c).add(d).doubleValue()));
            }
        }

        return path;
    }

    public static BigDecimal [][] rref(BigDecimal[][] mat) {
        int lead = 0;
        for (int r = 0; r < mat.length; r++) {
            int i = r;
            while (mat[i][lead].compareTo(BigDecimal.ZERO) == 0) {
                i++;
                if (mat.length == i) {
                    i = r;
                    lead++;
                }
            }

            BigDecimal [] tmp = mat[i];
            mat[i] = mat[r];
            mat[r] = tmp;

            BigDecimal val = mat[r][lead];
            for (int j = 0; j < mat[0].length; j++) {
                mat[r][j] = mat[r][j].divide(val, MathContext.DECIMAL64);
            }

            for (i = 0; i < mat.length; i++) {
                if (i == r) continue;
                val = mat[i][lead];
                for (int j = 0; j < mat[0].length; j++) {
                    mat[i][j] = mat[i][j].subtract(val.multiply(mat[r][j], MathContext.DECIMAL64), MathContext.DECIMAL64);
                }
            }
            lead++;
        }
        return mat;
    }

}
