package com.android.msckfs.utils;


import org.apache.commons.numbers.quaternion.Quaternion;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

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


    // own work.
    public static SimpleMatrix quaternionToVector(Quaternion q) {
        return null; // TODO: new ArrayRealVector(new double[]{q.getX(), q.getY(), q.getZ(), q.getScalarPart()});
    }

    // own work.
    public static Quaternion vectorToQuaternion(SimpleMatrix v) {
        assert(v.getNumCols() == 1);
        return Quaternion.of(v.get(3), v.get(0), v.get(1), v.get(2));
    }
    public static DMatrixRMaj quaternionToRotation(Quaternion q) {
        return null; // TODO
    }


    public static Quaternion rotationToQuaternion(DMatrixRMaj mat) {return null;} // TODO

    public static Quaternion smallAngleQuaternion(SimpleMatrix vec) {
        return null;
    }
    public static DMatrixRMaj skewSymmetric(DMatrixRMaj mat) {
        return null; //TODO
    }

    public static SimpleMatrix skewSymmetric(SimpleMatrix mat) {
        return null; //TODO
    }



}
