/*
 * @file CtcDynDist.cpp
 * @brief Implementation of the CtcDynDist dynamic Euclidean distance contractor.
 *
 * This file contains the implementation of the CtcDynDist class, a custom
 * contractor derived from codac::Ctc and adapted from CtcDist.
 *
 * The contractor enforces a Euclidean distance constraint between:
 *   - a static 2D point ([x1], [x2]), and
 *   - a dynamic 2D point ([px], [py]) = [p]([t]) extracted from a trajectory.
 *
 * Key implementation notes:
 *   - The trajectory is assumed to be thin (without uncertainty) and only the
 *     first two components are used.
 *   - The distance interval (range) is fixed at construction time and not
 *     modified by the contractor.
 *   - Forward and backward contraction are implemented using CODAC interval
 *     arithmetic (sqr, bwd_sqr, intersection, inversion).
 *
 * See CtcDynDist.h for full class and method documentation.
*/

#include "CtcDynDist.h"
#include <iostream>

/*
 * @brief Construct a CtcDynDist contractor.
 *
 * Performs sanity checks and initialization:
 *   - Ensures the trajectory has at least 2 dimensions.
 *     Only the first two components are used; if more are present and
 *     overpass_warning is false, a warning is printed.
 *   - Ensures the distance interval (range) is non-negative.
 *
 * These checks guarantee that the contractor can safely perform forward
 * and backward contraction on the spatial and temporal variables.
 *
 * @throws std::range_error if any of the input conditions are violated.
*/
CtcDynDist::CtcDynDist(const TubeVector &traj, const Interval &range, bool overpass_warning): 
_traj(traj), 
_range(range),
_overpass_warning(overpass_warning), 
Ctc(3)
{
    if (traj.size()<2) // ensure trajectory has at least 2 dimensions
    {
        throw std::range_error("[ERROR] CtcDynDist::CtcDynDist(const TubeVector &traj, const Interval &range, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 2");
    }

    if (traj.size()>2 && !overpass_warning)
    {
        std::cerr << "[WARNING] CtcDynDist::CtcDynDist(const TubeVector &traj, const Interval &range, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
    }
    
    if (!range.is_subset(Interval(0,oo))) // ensure distance is non-negative
    {
        throw std::range_error("[ERROR] CtcDynDist::CtcDynDist(const TubeVector &traj, const Interval &range, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
    }
}

/*
 * @brief Core contraction routine enforcing the dynamic Euclidean distance.
 *
 * Performs contraction between:
 *   - a static 2D point ([x1],[x2])
 *   - a dynamic 2D point ([px],[py]) = _traj([t])
 *
 * Implementation details:
 * 1. Forward contraction:
 *    - Computes the trajectory intervals at the given time t.
 *    - Calculates the squared differences between static and dynamic points.
 *    - Intersects with the distance interval (_range) to reduce uncertainty.
 *
 * 2. Backward contraction:
 *    - Propagates constraints back to [x1] and [x2] using bwd_sqr.
 *    - Updates intervals to respect the Euclidean distance constraint.
 *
 * 3. Temporal contraction:
 *    - Builds a size-adapted trajectory vector for inversion.
 *    - Uses _traj.invert(...) to contract the time interval t based on
 *      current x,y intervals.
 *
 * Assumptions:
 *  - The trajectory is thin (no uncertainty) and only the first two components
 *    are used.
 *  - Distance interval (_range) is fixed.
*/
void CtcDynDist::contract(Interval &x1, Interval &x2, Interval &t)
{
    if (!t.is_subset(_traj.tdomain())) // ensure t is coherent with the trajectory
    {
        throw std::range_error("[ERROR] CtcDynDist::contract(Interval &x1, Interval &x2, Interval &t): The input Interval t must be a subset of trajectory time domain");
    }

    // Create a local copy of the range to not contract it
    Interval range = _range;

    // FORWARD contraction
    // Compute trajectory positions at time t
    Interval px = _traj(t)[0]; // x-coordinate from trajectory
    Interval py = _traj(t)[1]; // y-coordinate from trajectory

    // Compute squared differences between static and dynamic points
    Interval i1 = -x1;
    Interval i2 = px + i1;
    Interval i3 = sqr(i2);

    Interval i4 = -x2;
    Interval i5 = py + i4;
    Interval i6 = sqr(i5);

    Interval i7 = i3 + i6;
    
    // Intersect with distance interval
    range &= sqrt(i7);
    
    // ----------------------------
    // BACKWARD contraction
    // Propagate constraints back to x1 and x2 using bwd_sqr// BACKWARD
    i7 &= sqr(range);

    i6 &= i7 - i3;
    i3 &= i7 - i6;

    bwd_sqr(i6,i5);
    i4 &= i5 - py;
    py &= i5 - i4;
    x2 &= -i4;

    bwd_sqr(i3,i2);
    i1 &= i2 - px;
    px &= i2 - i1;
    x1 &= -i1;

    // ----------------------------
    // TIME contraction
    // Adjust t based on contracted x and y
    IntervalVector size_adapted_p(_traj.size());
    size_adapted_p[0] = px;
    size_adapted_p[1] = py;

    t &= _traj.invert(size_adapted_p); // contract t using trajectory inversion
}

/*
 * @brief Convenience wrapper contracting a 2D point stored in an IntervalVector.
 *
 * Forwards the contraction to the core method:
 *    contract(x[0], x[1], t)
 *
 * Notes:
 *  - Only the first two components of x are used.
 *  - Any warnings or exceptions are handled in the core method.
*/
void CtcDynDist::contract(IntervalVector &x, Interval &t)
{
    if (x.size()<2)
    {
        throw std::range_error("[ERROR] CtcDynDist::contract(IntervalVector &x, Interval &t): IntervalVector x must have a dimension greater than or equal to 2");
    }

    if (x.size()>2 && !_overpass_warning)
    {
        std::cerr << "[WARNING] CtcDynDist::contract(IntervalVector &x, Interval &t): IntervalVector x has a dimension greater than 2, only the first two components will be used" << std::endl;
    }

    if (!t.is_subset(_traj.tdomain()))
    {
        throw std::range_error("[ERROR] CtcDynDist::contract(IntervalVector &x, Interval &t): The input Interval t must be a subset of trajectory time domain");
    }
    
    this->contract(x[0],x[1],t); // forward to main contract method
}

/*
 * @brief Convenience wrapper contracting a stacked vector ([x1],[x2],[t]).
 *
 * Forwards the contraction to the core method:
 *    contract(h[0], h[1], h[2])
 *
 * Notes:
 *  - Only the first three components of h are used.
 *  - This interface allows using a single IntervalVector for x1, x2, and t.
 *  - All warnings or exceptions are handled in the core method.
*/
void CtcDynDist::contract(IntervalVector &h)
{
    if (h.size()<3)
    {
        throw std::range_error("[ERROR] CtcDynDist::contract(IntervalVector &h): IntervalVector h must have a dimension greater than or equal to 3");
    }

    if (h.size()>3 && !_overpass_warning)
    {
        std::cerr << "[WARNING] CtcDynDist::contract(IntervalVector &h): IntervalVector h has a dimension greater than 3, only the first three components will be used" << std::endl;
    }
    
    if (!h[2].is_subset(_traj.tdomain()))
    {
        throw std::range_error("[ERROR] CtcDynDist::contract(IntervalVector &h): The third element of h must be a subset of trajectory time domain");
    }

    this->contract(h[0],h[1],h[2]); // forward to main contract method
}