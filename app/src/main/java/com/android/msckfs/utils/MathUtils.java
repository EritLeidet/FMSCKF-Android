package com.android.msckfs.utils;


import static org.ejml.dense.row.NormOps_DDRM.normF;
import static java.lang.Math.sqrt;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * <a href="https://github.com/rohiitb/msckf_vio_python/blob/main/utils.py">...</a>
 * <a href="https://github.com/KumarRobotics/msckf_vio/blob/master/include/msckf_vio/math_utils.hpp">...</a>
 */
// JPL Convention used by my vector representation of quaternions: [x, y, z, w(scalar)]^T
// Convention used by Apache Commons Quaternion class: [w(scalar),x,y,z]
public class MathUtils {

    // own work.
    public static DMatrixRMaj scale(double alpha, DMatrixRMaj m) {
        DMatrixRMaj out = new DMatrixRMaj();
        CommonOps_DDRM.scale(alpha, m, out);
        return out;
    }

    // own work.
    /**
     * @param start - inclusive
     * @param end - exclusive
     * @return SimpleMatrix without the selected rows.
     */
    public static SimpleMatrix deletedRows(SimpleMatrix m, int start, int end) {
        SimpleMatrix upperRows = m.rows(0, start);
        SimpleMatrix lowerRows = m.rows(end, SimpleMatrix.END);
        return m.concatRows(upperRows, lowerRows);

    }

    // own work.
    /**
     * @param start - inclusive
     * @param end - exclusive
     * @return SimpleMatrix without the selected columns.
     */
    public static SimpleMatrix deleteColumns(SimpleMatrix m, int start, int end) {
        SimpleMatrix leftCols = m.cols(0, start);
        SimpleMatrix rightCols = m.cols(end, SimpleMatrix.END);
        return m.concatColumns(leftCols, rightCols);

    }
    


    /**
     * Convert a quaternion to the corresponding rotation matrix.
     * The input quaternion should be in the form
     * [q1, q2, q3, q4(scalar)]^T
     */
    public static DMatrixRMaj quaternionToRotation(DMatrixRMaj q) {
        assert(q.numCols == 1 && q.numRows == 4);
        quaternionNormalize(q);
        SimpleMatrix qVec = new SimpleMatrix(new double[]{q.get(0), q.get(1), q.get(2)});
        final double q4 = q.get(3);
        return SimpleMatrix.identity(3)
                .scale(2*q4*q4-1)
                .minus(skewSymmetric(qVec).scale(2*q4))
                .plus(qVec.scale(2).mult(qVec.transpose())).getDDRM();
    }

    public static SimpleMatrix quaternionToRotation(SimpleMatrix q) {
        return SimpleMatrix.wrap(quaternionToRotation(q.getDDRM()));
    }


    /**
     * Convert a rotation matrix to a quaternion.
     * Pay attention to the convention used.
     * The input quaternion should be in the form
     * [q1, q2, q3, q4(scalar)]^T
     */
    public static SimpleMatrix rotationToQuaternion(SimpleMatrix R) {
        List<Double> score = new ArrayList<>();
        score.add(R.get(0,0));
        score.add(R.get(1,1));
        score.add(R.get(2,2));
        score.add(R.trace());

        int maxRow = score.indexOf(Collections.max(score));
        SimpleMatrix q = new SimpleMatrix(4,1);
        if (maxRow == 0) {
            q.set(0, sqrt(1+2*R.get(0,0)-R.trace()) / 2.0);
            q.set(1, (R.get(0,1)+R.get(1,0)) / (4*q.get(0)));
            q.set(2, (R.get(0,2)+R.get(2,0)) / (4*q.get(0)));
            q.set(3, (R.get(1,2)-R.get(2,1)) / (4*q.get(0)));
        } else if (maxRow == 1) {
            q.set(1, sqrt(1+2*R.get(1,1)-R.trace()) / 2.0);
            q.set(0, (R.get(0,1)+R.get(1,0)) / (4*q.get(1)));
            q.set(2, (R.get(1,2)+R.get(2,1)) / (4*q.get(1)));
            q.set(3, (R.get(2,0)-R.get(0,2)) / (4*q.get(1)));
        } else if (maxRow == 2) {
            q.set(2, sqrt(1+2*R.get(2,2)-R.trace()) / 2.0);
            q.set(0, (R.get(0,2)+R.get(2,0)) / (4*q.get(2)));
            q.set(1, (R.get(1,2)+R.get(2,1)) / (4*q.get(2)));
            q.set(3, (R.get(0,1)-R.get(1,0)) / (4*q.get(2)));
        } else {
            q.set(3, sqrt(1+R.trace()) / 2.0);
            q.set(0, (R.get(1,2)-R.get(2,1)) / (4*q.get(3)));
            q.set(1, (R.get(2,0)-R.get(0,2)) / (4*q.get(3)));
            q.set(2, (R.get(0,1)-R.get(1,0)) / (4*q.get(3)));
        }

        if (q.get(3) < 0) q = q.negative();

        return quaternionNormalize(q);
    }

    /**
     * Convert the vector part of a quaternion to a
     * full quaternion.
     * Note: This function is useful to convert delta quaternion
     *    which is usually a 3x1 vector to a full quaternion.
     *    For more details, check Section 3.2 "Kalman Filter Update" in
     *    "Indirect Kalman Filter for 3D Attitude Estimation:
     *    A Tutorial for quaternion Algebra".
     **/
    public static SimpleMatrix smallAngleQuaternion(SimpleMatrix dtheta) {
        assert(dtheta.getNumRows() == 3 && dtheta.getNumCols() == 1);
        SimpleMatrix dq = dtheta.divide(2.0);
        SimpleMatrix q = new SimpleMatrix(4,1);
        double dqSquareNorm = dq.dot(dq);

        q.insertIntoThis(0,0,dq);
        if (dqSquareNorm <= 1) {
            q.set(3, sqrt(1-dqSquareNorm));
        } else {
            q.set(3, 1);
            q = q.divide(sqrt(1+dqSquareNorm));
        }
        return q;
    }


    /**
     * Normalize the given quaternion to unit quaternion.
     **/
    public static SimpleMatrix quaternionNormalize(SimpleMatrix q) {
        return q.divide(q.normF());
    }

    public static void quaternionNormalize(DMatrixRMaj q) {
        CommonOps_DDRM.divide(q, normF(q));
    }

    /**
     * Convert a rotation matrix to a quaternion.
     * Pay attention to the convention used. The function follows the
     * conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
     * A Tutorial for Quaternion Algebra", Equation (78).
     * The input quaternion should be in the form [q1, q2, q3, q4(scalar)]
     */
    public static SimpleMatrix quaternionMultiplication(SimpleMatrix q1, SimpleMatrix q2) {
        q1 = quaternionNormalize(q1);
        q2 = quaternionNormalize(q2);
        SimpleMatrix L = new SimpleMatrix(new double[][]{
                {q1.get(3), q1.get(2),  -q1.get(1), q1.get(0)},
                {-q1.get(2),q1.get(3),  q1.get(0),  q1.get(1)},
                {q1.get(1), -q1.get(0), q1.get(3),  q1.get(2)},
                {-q1.get(0),-q1.get(1), -q1.get(2), q1.get(3)}
        });

        SimpleMatrix q = L.mult(q2);
        return quaternionNormalize(q);

    }

    /**
     * Rotation quaternion from v0 to v1.
     */

    public static SimpleMatrix fromTwoVectors(SimpleMatrix v0, SimpleMatrix v1) {
        v0 = v0.divide(v0.normF());
        v1 = v1.divide(v1.normF());
        double d = v0.dot(v1);

        // if dot == -1, vectors are nearly opposite
        SimpleMatrix q = new SimpleMatrix(4,1);
        if (d < -0.999999) {
            SimpleMatrix axis = cross(
                    new SimpleMatrix(new double[]{1,0,0}),
                    v0);
            if (axis.normF() < 0.000001) {
                axis = cross(
                        new SimpleMatrix(new double[]{0,1,0}),
                        v0);
                q.insertIntoThis(0,0,axis);
            }
        } else if (d > 0.999999) {
            q = new SimpleMatrix(new double[]{0,0,0,1});
        } else {
            double s = sqrt((1+d)*2);
            SimpleMatrix axis = cross(v0,v1);
            SimpleMatrix vec = axis.divide(s);
            double w = 0.5 * s;
            q.insertIntoThis(0,0,vec);
            q.set(3,w);
        }

        q = quaternionNormalize(q);
        return quaternionConjugate(q); // hamilton -> JPL
    }

    public static SimpleMatrix quaternionConjugate(SimpleMatrix q) {
        SimpleMatrix con = new SimpleMatrix(4,1);
        con.insertIntoThis(0,0, q.negative().rows(0,3));
        con.set(3, q.get(3));
        return con;

    }

    public static SimpleMatrix cross(SimpleMatrix a, SimpleMatrix b) {
        assert(a.isVector() && a.getNumRows() == 3);
        assert(b.isVector() && b.getNumRows() == 3);
        return new SimpleMatrix(new double[]{
                a.get(2)*b.get(3) - a.get(3)*b.get(2),
                a.get(3)*b.get(1) - a.get(1)*b.get(3),
                a.get(1)*b.get(2) - a.get(2)*b.get(1),
        });
    }

    /**
     * Create a skew-symmetric matrix from a 3-element vector.
     * w   ->  [  0 -w3  w2]
     *         [ w3   0 -w1]
     *         [-w2  w1   0]
     */
    public static DMatrixRMaj skewSymmetric(DMatrixRMaj mat) {
        assert(mat.numCols == 1 && mat.numRows == 3);
        double x = mat.get(0);
        double y = mat.get(1);
        double z = mat.get(2);
        return new DMatrixRMaj(new double[][]{
                {0, -z, y},
                {z, 0, -x},
                {-y, x, 0}
        });
    }


    public static SimpleMatrix skewSymmetric(SimpleMatrix mat) {
        return SimpleMatrix.wrap(skewSymmetric(mat.getDDRM()));
    }



}
