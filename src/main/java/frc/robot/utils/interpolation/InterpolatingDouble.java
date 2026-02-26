package frc.robot.utils.interpolation;

/**
 * A Double that can be interpolated using the InterpolatingTreeMap
 * Basically a wrapper that supports Interpolable & InverseInterpolable
 *
 * @see InterpolatingTreeMap
 */
public class InterpolatingDouble implements Interpolable<InterpolatingDouble>, InverseInterpolable<InterpolatingDouble>,
        Comparable<InterpolatingDouble> {

    // The numeric value being wrapped
    public Double value = 0.0;

    // Constructs the InterpolatingDouble with a given Value
    public InterpolatingDouble(Double val) {
        value = val;
    }

    /**
    * Linearly interpolates between this Value (lower bound) and another Value (upper bound)
    * @param other  the upper bound Value
    * @param x  the interpolation factor
    * @return a new InterpolatingDouble representing the interpolated value
    * */
    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double x) {
        // Difference between upper and lower values
        Double dydx = other.value - value;

        // Applying the linear interpolation formula: y = y0 + (y1 - y0) * x
        Double searchY = dydx * x + value;

        return new InterpolatingDouble(searchY);
    }

    /**
     * Determines how far a query value lies between this Value (lower bound)
     * and an upper bound Value
     *
     * This is the inverse operation of interpolate()
     *
     * @param upper the upper bound value
     * @param query the value being searched for
     * @return a ratio between 0.0 and 1.0 representing relative position
     */
    @Override
    public double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
        // Distance between upper and lower bounds
        double upper_to_lower = upper.value - value;
        if (upper_to_lower <= 0) {
            return 0;
        }

        // Distance from lower bound to query value
        double query_to_lower = query.value - value;
        if (query_to_lower <= 0) {
            return 0;
        }

        // Ratio representing how far query is between lower and upper
        return query_to_lower / upper_to_lower;
    }


    /**
     * Compares this InterpolatingDouble to another for ordering
     *
     * @param other another InterpolatingDouble to compare against
     * @return positive 1 if greater, negative 1 if less, zero if equal
     */
    @Override
    public int compareTo(InterpolatingDouble other) {
        return value.compareTo(other.value);
    }

}