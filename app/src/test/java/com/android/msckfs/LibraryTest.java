package com.android.msckfs;


import org.ejml.data.DMatrixSparseCSC;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

public class LibraryTest {
    @Test
    public void multDifferentSimpleMatrixTypes() {
        SimpleMatrix A = SimpleMatrix.random_DDRM(1,3);
        SimpleMatrix B = SimpleMatrix.wrap(new DMatrixSparseCSC(3,1));
        SimpleMatrix C = A.mult(B);
        System.out.println(C.getType());
    }
}
