/*
 * @file CtcDynPolar.cpp
 * @brief Implementation of the CtcDynPolar dynamic polar contractor.
 *
 * This file contains the implementation of the CtcDynPolar class, a custom
 * contractor derived from codac::Ctc and adapted from the static CtcPolar
 * contractor.
 *
 * The contractor enforces a polar constraint between:
 *   - a static 2D point ([x1], [x2]), and
 *   - a dynamic pose ([px], [py], [theta]) obtained from a trajectory.
 *
 * The polar constraint models an oriented range sensor whose perception is
 * limited both in distance ([range]) and in angle ([delta]) relative to the
 * robot heading.
 *
 * Key implementation notes:
 *   - The trajectory is assumed to be thin (without uncertainty) and is not
 *     contracted.
 *   - Only the first three components of the trajectory are used:
 *       x-position, y-position, heading.
 *   - The angular interval [delta] is normalized modulo 2π to avoid angle
 *     wrapping issues.
 *   - Contraction is performed on spatial variables ([x1], [x2]) and on the
 *     temporal variable ([t]).
 *
 * See CtcDynPolar.h for full class and method documentation.
 */

#include "CtcDynPolar.h"
#include <iostream>

/*==================================================================================
 * INTERNAL UTILITIES
 *==================================================================================*/

namespace 
{
    /*
     * @brief Normalize an angular interval modulo 2π.
     *
     * This function maps an angular interval to an equivalent interval whose
     * mean lies within [-π, π]. It is used to avoid angular discontinuities
     * when dealing with polar constraints.
     *
     * If the width of the interval is greater than or equal to 2π, the angle
     * fully wraps around the circle and the result is set to [-π, π].
     *
     * @param delta Angular interval to normalize.
     * @return Interval equivalent to delta modulo 2π.
     */
    Interval modulo2pi(const Interval delta)
    {
        // If the field of view spans the full circle (or more),
        // angular constraints are meaningless
        if (delta.ub()-delta.lb()>=2*M_PI)
        {
            // Return the full angular domain [-π, π]
            return (Interval(0)|Interval(Interval::TWO_PI)) - Interval::PI;
        }
        
        // Otherwise, ensure that the interval mean lies within [-π, π]
        Interval mod_delta(delta);
        double mean_angle = (mod_delta.ub()+mod_delta.lb())/2.;

        // Shift interval by multiples of 2π until mean is within bounds
        while(mean_angle>M_PI)
        {
            mod_delta = mod_delta - 2*M_PI;
            mean_angle = (mod_delta.ub()+mod_delta.lb())/2.;
        }
        
        // Shift interval by multiples of -2π until mean is within bounds
        while(mean_angle<-M_PI)
        {
            mod_delta = mod_delta + 2*M_PI;
            mean_angle = (mod_delta.ub()+mod_delta.lb())/2.;
        }

        // Return the interval delta modulo 2π and centered on [-π, π]
        return mod_delta;
    }
}

/*==================================================================================
 * CONSTRUCTOR
 *==================================================================================*/

/*
 * @brief Construct a CtcDynPolar contractor.
 *
 * Performs sanity checks and initialization:
 *   - Ensures the trajectory has at least 3 dimensions
 *     (x, y, heading).
 *   - Warns if the trajectory has more than 3 dimensions
 *     (only the first three are used).
 *   - Ensures the range interval is non-negative.
 *   - Normalizes the angular field of view modulo 2π.
 *   - Warns the user if the angular aperture spans 2π or more,
 *     in which case CtcDynDist would be more efficient.
 *
 * @throws std::range_error if input conditions are violated.
 */
CtcDynPolar::CtcDynPolar(const TubeVector &traj, const Interval range, const Interval delta, bool overpass_warning): 
_traj(traj), 
_range(range), 
_delta(modulo2pi(delta)),
_overpass_warning(overpass_warning),
Ctc(3) 
{
    // Trajectory must provide (x, y, heading)
    if (traj.size()<3)
    {
        throw std::range_error("[ERROR] CtcDynPolar::CtcDynPolar(const TubeVector &traj, const Interval &range, const Interval &delta, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
    }
    
    // Warn if extra dimensions are present
    if (traj.size()>3)
    {
        std::cerr<<"[WARNING] CtcDynPolar::CtcDynPolar(const TubeVector &traj, const Interval &range, const Interval &delta, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used"<<std::endl;
    }

    // Range must be non-negative
    if (!range.is_subset(Interval(0,oo)))
    {
        throw std::range_error("[ERROR] CtcDynPolar::CtcDynPolar(const TubeVector &traj, const Interval &range, const Interval &delta, bool overpass_warning): The input Interval range must be a subset inside R+");
    }
    
    // Warn if angular constraint spans full circle
    if (delta.ub()-delta.lb()>=2*M_PI && !_overpass_warning)
    {
        std::cerr<<"[WARNING] CtcDynPolar::CtcDynPolar(const TubeVector &traj, const Interval &range, const Interval &delta, bool overpass_warning): You initialized an instance of class CtcDynPolar with an interval delta with a width superior or equal to 2*PI. Please use the CtcDynDist class which is dedicated for this case."<<std::endl;
    }
}

/*==================================================================================
 * CORE CONTRACTION METHOD
 *==================================================================================*/

/*
 * @brief Core contraction routine enforcing the dynamic polar constraint.
 *
 * Enforces a polar constraint between:
 *   - a static point ([x1], [x2]), and
 *   - a dynamic pose ([px], [py], [theta]) = _traj([t]).
 *
 * The contraction is performed on:
 *   - spatial variables [x1], [x2],
 *   - temporal variable [t].
 *
 * The trajectory itself is assumed to be thin and is not contracted.
 */
void CtcDynPolar::contract(Interval &x1, Interval &x2, Interval &t)
{
    // Ensure time interval is consistent with trajectory
    if (!t.is_subset(_traj.tdomain()))
    {
        throw std::range_error("[ERROR] CtcDynPolar::contract(Interval &x1, Interval &x2, Interval &t): The input Interval t must be a subset the trajectory time domain");
    }

    // Local copies to avoid contracting fixed parameters
    Interval range(_range);
    Interval delta(_delta);

    // ==========================
    // FORWARD propagation
    // ==========================

    // Extract trajectory pose at time t
    Interval px = _traj(t)[0];
    Interval py = _traj(t)[1];
    Interval theta = _traj(t)[2];

    // Express static point in robot local frame
    Interval i1 = x1 - px;
    Interval i2 = x2 - py;

    // Relative bearing interval
    Interval i3 = theta + delta;

    // ==========================
    // POLAR contraction
    // ==========================

    CtcPolar ctc_polar;
    ctc_polar.contract(i1, i2, range, i3);
    
    // ==========================
    // BACKWARD propagation
    // ==========================

    // Propagate constraints back to world coordinates
    x1 &= i1 + px;
    px &= x1 - i1;

    x2 &= i2 + py;
    py &= x2 - i2;

    // Propagate constraints back to heading
    theta &= i3 - delta;
    delta &= i3 - theta; 

    // ==========================
    // TIME contraction
    // ==========================

    // Build size-adapted vector for trajectory inversion
    IntervalVector size_adapted_p(_traj.size());
    size_adapted_p[0] = px;
    size_adapted_p[1] = py;
    size_adapted_p[2] = theta;

    // Contract time using trajectory inversion
    t &= _traj.invert(size_adapted_p);   
}

/*==================================================================================
 * CONVENIENCE WRAPPERS
 *==================================================================================*/

/*
 * @brief Convenience wrapper contracting a 2D point stored in an IntervalVector.
 *
 * Forwards the contraction to:
 *   contract(x[0], x[1], t)
 */
void CtcDynPolar::contract(IntervalVector &x, Interval &t)
{
    if (x.size()<2)
    {
        throw std::range_error("[ERROR] CtcDynPolar::contract(IntervalVector &x, Interval &t): The input IntervalVector x must have a dimension greater than 2");
    }
    if (x.size()>2 && !_overpass_warning)
    {
        std::cerr<<"[WARNING] CtcDynPolar::contract(IntervalVector &x, Interval &t): The input IntervalVector x has a dimension greater than 2, only the first two components will be used"<<std::endl;
    }

    if (!t.is_subset(_traj.tdomain()))
    {
        throw std::range_error("[ERROR] CtcDynPolar::contract(IntervalVector &x, Interval &t): The input Interval t must be a subset of the trajectory time domain");
    }

    this->contract(x[0],x[1],t);
}

/*
 * @brief Convenience wrapper contracting a stacked vector ([x1], [x2], [t]).
 *
 * Forwards the contraction to:
 *   contract(h[0], h[1], h[2])
 */
void CtcDynPolar::contract(IntervalVector &h)
{
    if (h.size()<3)
    {
        throw std::range_error("[ERROR] CtcDynPolar::contract(IntervalVector &h): The input IntervalVector h must have a dimension greater than 3");
    }
    if (h.size()>3 && !_overpass_warning)
    {
        std::cerr<<"[WARNING] CtcDynPolar::contract(IntervalVector &h): The input IntervalVector h has a dimension greater than 3, only the first three components will be used"<<std::endl;
    }

    if (!h[2].is_subset(_traj.tdomain()))
    {
        throw std::range_error("[ERROR] CtcDynPolar::contract(IntervalVector &h): The third element of h must be a subset of trajectory time domain");
    }

    this->contract(h[0],h[1],h[2]);
}