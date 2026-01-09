/*
 * @file SepDynPolar.h
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 * 
 * @brief Local separator for polar-based perception along a dynamic trajectory.
 *
 * This class defines a local separator that models the perception of a static
 * 2D point by a mobile robot equipped with an oriented range sensor following
 * a known (thin) trajectory, based on distance and angular constraints.
 *
 * The separator acts on a 3-dimensional box (x, y, t) and separates it according
 * to whether the point is perceived by the robot at time t.
 *
 * IMPORTANT:
 * -----------
 * SepDynPolar is a local separator. It does NOT compute the covered area over
 * an entire trajectory by itself. To compute a global covered area, this separator
 * must be projected over time using tools such as SepProj, or integrated into a
 * higher-level class dedicated to coverage computation.
 *
 * Using a time interval equal to the full trajectory domain directly with this
 * separator will generally lead to over-approximation, because tube evaluation
 * over a time interval loses temporal correlation.
 *
 * --------------------------------------------------------------------------
 * Perception models
 * --------------------------------------------------------------------------
 * This class supports TWO different perception logics, depending on the
 * constructor used.
 *
 * --------------------------------------------------------------------------
 * 1) Binary perception logic
 * --------------------------------------------------------------------------
 * When the separator is defined using a single distance interval `range` and
 * a single angular interval `delta`, perception is modeled using a binary logic:
 *
 *   - Distance ∈ range AND angle ∈ delta → the point is considered covered
 *   - Otherwise → the point is considered not covered
 *
 * --------------------------------------------------------------------------
 * 2) Three-valued perception logic
 * --------------------------------------------------------------------------
 * When the separator is defined using both `range`/`uncertain_range` and
 * `delta`/`uncertain_delta`, perception is modeled using three-valued logic:
 *
 *   - Distance ∈ range AND angle ∈ delta
 *       → covered for sure
 *   - Distance ∈ (uncertain_range \ range) OR angle ∈ (uncertain_delta \ delta)
 *       → unknown (may or may not have been perceived)
 *   - Distance ∉ uncertain_range OR angle ∉ uncertain_delta
 *       → not covered for sure
 *
 * --------------------------------------------------------------------------
 * Geometric interpretation
 * --------------------------------------------------------------------------
 * At a given time t, the robot is located at p(t) with heading θ(t). The perception
 * region is a sector (or pie) defined by:
 *
 *   - Radius: inner r_min and outer r_max
 *   - Angle: [θ - delta/2, θ + delta/2]
 *
 * If r_min = 0, the annular sector degenerates into a circular sector (disk-shaped).
 * If delta spans the full circle (≥2π), the angular constraint is meaningless, and
 * the perception region becomes a ring (use SepDynDist for better performance).
 *
 * --------------------------------------------------------------------------
 * Variables and assumptions
 * --------------------------------------------------------------------------
 * - The separator acts on an IntervalVector of dimension 3: (x, y, t)
 * - The trajectory is assumed to be thin
 * - No contraction is performed on the trajectory itself
 *
 * --------------------------------------------------------------------------
 * Implementation note on SepCtcPair
 * --------------------------------------------------------------------------
 * Internally, this separator relies on SepCtcPair.
 * In SepCtcPair, the first contractor corresponds to the INSIDE set and the
 * second one to the OUTSIDE set.
 *
 * Due to this convention, internal variable names (_ctcin, _ctcout) may appear
 * counter-intuitive at first glance. The semantic behavior of the separator,
 * however, strictly follows the definitions given above.
 *
 * --------------------------------------------------------------------------
 * Typical usage
 * --------------------------------------------------------------------------
 * - Local separation of (x, y, t) boxes using a polar sensor model
 * - Projection over time using SepProj to compute covered areas
 * - Integration in paving algorithms for guaranteed coverage analysis
 *
 * This class is part of the codac-coverage-toolbox and is intended for
 * guaranteed, set-based coverage and perception analysis.
 */

#ifndef __SEP_DYN_POLAR_H___
#define __SEP_DYN_POLAR_H___

#include "codac.h"
#include "CtcDynPolar.h"


using namespace codac;

/*
 * @class SepDynPolar
 * @brief Local separator modeling polar-based perception along a trajectory.
 *
 * SepDynPolar is a local separator representing the perception of a static
 * 2D point by a mobile robot following a known trajectory, using a distance
 * and angular constraint between the robot position and the point at time t.
 *
 * The separator acts on a 3-dimensional IntervalVector (x, y, t), where:
 *   - (x, y) is the unknown position of the object,
 *   - t is the (possibly uncertain) detection time.
 *
 * For a given time t, the robot is at p(t) with heading θ(t), and the perception
 * region is a polar sector defined by radius and angle constraints.
 *
 * --------------------------------------------------------------------------
 * Local separator
 * --------------------------------------------------------------------------
 * SepDynPolar is local: it only describes the perception constraint at a given
 * time t (or over a small time interval). It does not compute the full covered
 * area along the trajectory directly.
 *
 * Use with SepProj to project the local separation over time for global coverage.
 */
class SepDynPolar: public SepCtcPair
{
    public:
        
        /*
        * @brief Constructor for binary perception logic (distance + angular interval).
        *
        * Points at a distance within `range` and angle within `delta` are considered
        * certainly covered. Points outside these intervals are considered not covered.
        *
        * @param traj Thin TubeVector representing the robot trajectory (must have ≥3 dimensions)
        * @param range Interval representing distance bounds: [r_min, r_max]
        * @param delta Angular interval relative to robot heading (centered on 0)
        * @param overpass_warning If true, suppresses warnings about trajectory dimension or degenerate ranges
        */
        SepDynPolar(const TubeVector& traj, const Interval range, const Interval delta, bool overpass_warning = false);
        
        /*
        * @brief Constructor for three-valued perception logic.
        *
        * Points within `range` and `delta` are covered for sure.
        * Points within `uncertain_range` or `uncertain_delta` but outside `range`/`delta` are uncertain.
        * Points outside `uncertain_range` or `uncertain_delta` are not covered.
        *
        * @param traj Thin TubeVector representing the robot trajectory (must have ≥3 dimensions)
        * @param range Interval representing certain distance perception
        * @param uncertain_range Interval representing uncertain distance perception
        * @param delta Angular interval for certain perception
        * @param uncertain_delta Angular interval for uncertain perception
        * @param overpass_warning If true, suppresses warnings about trajectory dimension or degenerate ranges
        */
        SepDynPolar(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning = false);
    private:
        
        // Contractor for guaranteed perceived region (inside polar sector)
        CtcDynPolar _ctcin;
        
        // Contractor for guaranteed not-perceived region (outside polar sector / complement)
        CtcUnion _ctcout;
};

# endif