/*
 * @file SepDynPie.h
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 * 
 * @brief Local separator for pie-shaped (disk sector) perception along a dynamic trajectory.
 *
 * This class defines a local separator that models the perception of a static
 * 2D point by a mobile vehicle equipped with an oriented range sensor following
 * a known (thin) trajectory, in the specific case where the perception range
 * starts at zero.
 *
 * The perception region is a pie-shaped sector (i.e., a circular sector),
 * defined by a maximum range and an angular field of view.
 *
 * The separator acts on a 3-dimensional box (x, y, t) and separates it according
 * to whether the point is perceived by the vehicle at time t.
 *
 * IMPORTANT:
 * -----------
 * SepDynPie is a local separator. It does NOT compute the covered area over
 * an entire trajectory by itself. To compute a global covered area, this separator
 * must be projected over time using tools such as SepProj, or integrated into a
 * higher-level class dedicated to coverage computation.
 *
 * Using a time interval equal to the full trajectory domain directly with this
 * separator will generally lead to over-approximation, because tube evaluation
 * over a time interval loses temporal correlation.
 *
 * --------------------------------------------------------------------------
 * Relation with SepDynPolar
 * --------------------------------------------------------------------------
 * SepDynPie is a specialized and optimized version of SepDynPolar for the case
 * where the minimum perception range r_min is equal to zero.
 *
 * Compared to SepDynPolar:
 *   - The perception region is always of the form [0, r_max] in distance
 *   - There is no inner "outside" distance region to consider
 *   - The outside set can therefore be computed using fewer contractors
 *
 * This reduction leads to improved performance, especially when the separator
 * is used repeatedly inside heavy algorithms such as SepProj.
 *
 * If a non-zero minimum range is required (r_min > 0), SepDynPolar must be used.
 *
 * --------------------------------------------------------------------------
 * Perception models
 * --------------------------------------------------------------------------
 * This class supports several perception configurations depending on the
 * constructor used.
 *
 * --------------------------------------------------------------------------
 * 1) Binary perception logic (range + angular sector)
 * --------------------------------------------------------------------------
 * When the separator is defined using a maximum range and an angular field
 * of view, perception is modeled using a binary logic:
 *
 *   - Distance ∈ [0, r_max] AND angle ∈ delta
 *       → the point is considered covered
 *   - Otherwise
 *       → the point is considered not covered
 *
 * The angular sector delta is always defined in the robot frame.
 *
 * --------------------------------------------------------------------------
 * 2) Three-valued perception logic (uncertain range and FOV)
 * --------------------------------------------------------------------------
 * When uncertain borders are provided, perception follows a three-valued logic:
 *
 *   - Distance ∈ [0, r_max] AND angle ∈ delta
 *       → covered for sure
 *   - Distance ∈ (r_max, r_max + range_border] OR angle ∈ (delta_border \ delta)
 *       → unknown (may or may not have been perceived)
 *   - Distance > r_max + range_border OR angle ∉ delta_border
 *       → not covered for sure
 *
 * This allows modeling sensor uncertainty on both range and field of view.
 *
 * --------------------------------------------------------------------------
 * Robot frame and angular conventions
 * --------------------------------------------------------------------------
 * All angular quantities are expressed in the robot frame:
 *
 *   - The robot frame origin is located at the vehicle position p(t)
 *   - The x-axis of the robot frame is aligned with the vehicle heading θ(t)
 *   - An angle equal to 0 corresponds to the forward direction of the vehicle
 *   - Positive angles are defined counterclockwise
 *
 * The angular interval delta therefore represents a field of view relative to
 * the vehicle heading.
 *
 * For constructors using (fov, fov_offset), the angular sector is defined as:
 *
 *   delta = [fov_offset − fov/2 , fov_offset + fov/2]
 *
 * The global bearing of a detected point at time t is then θ(t) + delta.
 *
 * --------------------------------------------------------------------------
 * Variables and assumptions
 * --------------------------------------------------------------------------
 * - The separator acts on an IntervalVector of dimension 3: (x, y, t)
 * - The problem is 2D; higher-dimensional trajectories are accepted but only
 *   the first three components (x, y, heading) are used
 * - The trajectory is assumed to be thin
 * - The trajectory itself is never contracted
 *
 * --------------------------------------------------------------------------
 * Implementation note on angular normalization
 * --------------------------------------------------------------------------
 * Angular intervals are internally normalized modulo 2π to ensure numerical
 * robustness and to handle user-defined angles consistently.
 *
 * This mechanism is mainly intended as a safety feature for advanced usage.
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
 * - Local separation of (x, y, t) boxes using a pie-shaped sensor model
 * - Projection over time using SepProj to compute covered areas
 * - Guaranteed coverage analysis for mobile vehicles with oriented range sensors
 *
 * This class is part of the codac-coverage-toolbox and is intended for
 * guaranteed, set-based coverage and perception analysis.
 */

#ifndef __SEP_DYN_PIE_H___
#define __SEP_DYN_PIE_H___

#include "codac.h"
#include "CtcDynPolar.h"

/*
 * @class SepDynPie
 * @brief Local separator modeling pie-shaped perception along a trajectory.
 *
 * SepDynPie represents the perception of a static 2D point by a mobile vehicle
 * following a known trajectory, equipped with an oriented range sensor whose
 * perception range starts at zero.
 *
 * The perception region at time t is a circular sector (pie) defined by:
 *   - A maximum range r_max
 *   - An angular field of view expressed in the robot frame
 *
 * The separator acts on a 3-dimensional IntervalVector (x, y, t), where:
 *   - (x, y) is the unknown position of the point
 *   - t is the (possibly uncertain) detection time
 *
 * SepDynPie is local and must be used with SepProj to compute global covered areas.
 */
using namespace codac;

class SepDynPie: public SepCtcPair
{
    public:

        /*
         * @brief Constructor using maximum range, field of view and angular offset.
         *
         * The perception sector is defined as:
         *   - Distance ∈ [0, range]
         *   - Angle ∈ [fov_offset − fov/2 , fov_offset + fov/2]
         *
         * All angles are expressed in the robot frame.
         *
         * @param traj Thin TubeVector representing the vehicle trajectory (≥ 3 dimensions)
         * @param range Maximum perception range r_max (must be > 0)
         * @param fov Full opening angle of the field of view
         * @param fov_offset Angular offset of the FOV center in the robot frame
         * @param overpass_warning If true, suppresses warnings about degenerate cases
         */
        SepDynPie(const TubeVector& traj, const double range, const double fov, const double fov_offset = 0., bool overpass_warning = false);
        
        /*
         * @brief Constructor using maximum range and explicit angular interval.
         *
         * The perception sector is defined as:
         *   - Distance ∈ [0, range]
         *   - Angle ∈ delta (expressed in the robot frame)
         *
         * @param traj Thin TubeVector representing the vehicle trajectory (≥ 3 dimensions)
         * @param range Maximum perception range r_max (must be > 0)
         * @param delta Angular interval in the robot frame
         * @param overpass_warning If true, suppresses warnings about degenerate cases
         */
        SepDynPie(const TubeVector& traj, const double range, const Interval delta, bool overpass_warning = false);
        
        /*
         * @brief Constructor for three-valued perception logic with uncertainty.
         *
         * This constructor allows modeling uncertainty on both range and field of view.
         *
         * @param traj Thin TubeVector representing the vehicle trajectory (≥ 3 dimensions)
         * @param range Maximum certain perception range r_max
         * @param range_border Additional uncertain range beyond r_max
         * @param delta Angular interval for certain perception
         * @param delta_border Angular interval for uncertain perception
         * @param overpass_warning If true, suppresses warnings about degenerate cases
         */
        SepDynPie(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, bool overpass_warning = false);
    
    private:

        // Contractor for guaranteed perceived region (inside the pie sector)
        CtcDynPolar _ctcin;

        // Contractor for guaranteed not-perceived region (outside the pie sector)
        CtcUnion _ctcout;
};

# endif