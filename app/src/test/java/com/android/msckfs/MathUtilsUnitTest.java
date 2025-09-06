package com.android.msckfs;


import static com.msckf.lib.utils.MathUtils.deleteColumns;
import static com.msckf.lib.utils.MathUtils.deleteRows;

import org.ejml.EjmlUnitTests;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

public class MathUtilsUnitTest {


    @Test
    public void testDeleteRows() {
        SimpleMatrix given = new SimpleMatrix(new double[][]{
                {1,2,3},
                {4,5,6},
                {7,8,9}
        });

        SimpleMatrix expected = new SimpleMatrix(new double[][]{
                {1,2,3},
                {7,8,9}
        });

        SimpleMatrix actual = deleteRows(given, 1,2);
        EjmlUnitTests.assertEquals(expected.getDDRM(), actual.getDDRM());

    }

    @Test
    public void testDeleteRowsCols() {
        SimpleMatrix given = new SimpleMatrix(new double[][]{
                {1,2,3},
                {4,5,6},
                {7,8,9}
        });

        SimpleMatrix expected = new SimpleMatrix(new double[][]{
                {1,3},
                {4,6},
                {7,9}
        });

        SimpleMatrix actual = deleteColumns(given, 1,2);
        EjmlUnitTests.assertEquals(expected.getDDRM(), actual.getDDRM());

    }
}
