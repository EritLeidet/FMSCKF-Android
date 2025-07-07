package com.android.msckfs.utils;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * Map that preserves insertion order. Similar to LinkedHashMap, whose functions are only available for API level 35+.
 * @param <K> key type
 * @param <V> value type
 */
public class LinkedDequeMap<K,V> {

    private Map<K,V> map;
    private LinkedList<K> list;

    public LinkedDequeMap() {
        this.map = new HashMap<>();
        this.keys = new LinkedList<>();
        this.values = new LinkedList<>();
    }

    public Map.Entry<K,V> get(K key) {
        return map.get(key);
    }

}
