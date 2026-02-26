package frc.robot.utils.interpolation;
import java.util.TreeMap;

/**
 * Interpolating Tree Maps are used to get values at points that are not defined by making a guess from points that are
 * defined. This uses linear interpolation.
 *
 * @param <K> The type of the Key (must implement InverseInterpolable)
 * @param <V> The type of the Value (must implement Interpolable)
 */
public class InterpolatingTreeMap<K extends InverseInterpolable<K> & Comparable<K>, V extends Interpolable<V>>
        extends TreeMap<K, V> {

    // max defines the interpolation's size limit
    int max;

    // Constructor basically takes one parameter, the maximum size of the tree
    public InterpolatingTreeMap(int maximumSize) {
        max = maximumSize;
    }

    // In case no maximum size is specified, it is set to 0
    public InterpolatingTreeMap() {
        this(0);
    }

    /**
     * Inserts a Key-Value pair (x-y pair), and trims the tree if a max size is specified
     *
     * @param key Key for inserted data
     * @param value Value for inserted data
     * @return the value
     */
    @Override
    public V put(K key, V value) {
        if (max > 0 && max <= size()) {
            // "Cuts down" the tree if it is oversize; i.e. if a size of 5 is specified and
            // we try to define a 6th element, then the 1st element will be trimmed out to
            // leave space for the 6th element and so on
            K first = firstKey();
            remove(first);
        }

        // Inserts the provided Key-Value pair into a higher-level tree
        super.put(key, value);

        return value;
    }

    /**
     *  Returns a predicted Value from a given Key (y from a given x)
     *
     * @param key Lookup for a value (does not have to exist)
     * @return V or null; V if it is Interpolable or exists, null if it is at a bound and cannot average
     */
    public V getInterpolated(K key) {
        // Checks whether the Value is actually defined inside the interpolation tree
        V gotval = get(key);

        // In case it is not, we predict it using surrounding Keys
        if (gotval == null) {
            // Get surrounding Keys
            K topBound = ceilingKey(key);
            K bottomBound = floorKey(key);

            // If attempting interpolation at ends of tree, return the nearest data point
            if (topBound == null && bottomBound == null) {
                return null;
            } else if (topBound == null) {
                return get(bottomBound);
            } else if (bottomBound == null) {
                return get(topBound);
            }

            // Get surrounding Values for interpolation
            V topElem = get(topBound);
            V bottomElem = get(bottomBound);

            // Returns gotten Value, with method declared in InterpolatingDouble
            return bottomElem.interpolate(topElem, bottomBound.inverseInterpolate(topBound, key));
        } else {
            // In case the Value is explicitly defined for a Key inside the interpolation tree, we
            // simply return it
            return gotval;
        }
    }
}

/*
* FTC Usage example in Kotlin:
*
*      // Declaring Tree Map
*       val map: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
*           = InterpolatingMapTree(maximumSize = 10)
*
*      // Adding data to the tree
*       map.put(
*           InterpolatingDouble(1.0.inches),
*           InterpolatingDouble(36.0.degrees)
*       )
* */