package com.android.msckfs.utils;


import org.apache.commons.numbers.quaternion.Quaternion;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

// JPL Convention used by my vector representation of quaternions: [x, y, z, w(scalar)]^T
// Convention used by Quaternion class: [w(scalar),x,y,z]
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
        SimpleMatrix lowerRows = m.rows(end, m.getNumRows());
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
        SimpleMatrix rightCols = m.cols(end, m.getNumCols());
        return m.concatColumns(leftCols, rightCols);

    }


    // own work.
    public static SimpleMatrix quaternionToVector(Quaternion q) {
        return new ArrayRealVector(new double[]{q.getX(), q.getY(), q.getZ(), q.getScalarPart()});
    }

    // own work.
    public static Quaternion vectorToQuaternion(SimpleMatrix v) {
        assert(v.getDimension() == 4);
        return Quaternion.of(v.getEntry(3), v.getEntry(0), v.getEntry(1), v.getEntry(2));
    }
    public static DMatrixRMaj quaternionToRotation(Quaternion q) {
        // TODO: different quaternion convention C++ / Java
        final RealVector qVec = MatrixUtils.createRealVector(new double[]{q.getX(), q.getY(), q.getZ(), q.getScalarPart()});
        final double q4 = q.getScalarPart();
        // TODO: operate instead of preMultiply?
        return MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(2*q4*q4-1).subtract(skewSymmetric(qVec).scalarMultiply(2*q4)).add(transpose(qVec).preMultiply(qVec.mapMultiply(2))); // TODO: instead of custom tranpose, convert to
    }

    public static Quaternion rotationToQuaternion(DMatrixRMaj mat) {return null;} // TODO

    public static Quaternion smallAngleQuaternion(SimpleMatrix vec) {
        return null;
    }
    public static DMatrixRMaj skewSymmetric(DMatrixRMaj mat) {
        return null; //TODO
    }



}
