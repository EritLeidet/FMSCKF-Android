package com.android.msckfs;


import org.ejml.data.DMatrixSparseCSC;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

/**
 * Ensure the Matrix library operations work as expected.
 */
public class LibraryTest {
    @Test
    public void multDifferentSimpleMatrixTypes() {
        SimpleMatrix A = SimpleMatrix.random_DDRM(1,3);
        SimpleMatrix B = SimpleMatrix.wrap(new DMatrixSparseCSC(3,1));
        SimpleMatrix C = A.mult(B);
        System.out.println(C.getType());
    }

    @Test
    public void addZeroVectors() {
        SimpleMatrix v1 = new SimpleMatrix(3,1);
        SimpleMatrix v2 = new SimpleMatrix(3,1);
        System.out.println(v1.plus(v2));
    }
}
