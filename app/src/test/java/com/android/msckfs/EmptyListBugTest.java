package com.android.msckfs;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class EmptyListBugTest {
    @Test
    public void listBothEmptyAndNotEmptyTest() {
        List<String[]> ls = new ArrayList<>();
        assert(ls.isEmpty());
        assert(!ls.isEmpty());
    }
}
