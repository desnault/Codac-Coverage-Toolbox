/*
 * @file SepDynDisk.h
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 *
 * @brief Local separator for disk-based perception along a dynamic trajectory.
 *
 * This class defines a local separator that models the perception of a static
 * 2D point by a mobile robot following a known (thin) trajectory, based on a
 * distance constraint between the object position and the robot position at
 * time t.
 *
 * The separator acts on a 3-dimensional box (x, y, t) and separates it according
 * to whether the object is perceived or not by the robot at time t.
 *
 * IMPORTANT:
 * -----------
 * SepDynDisk is a local separator. It does NOT compute the covered area over an
 * entire trajectory by itself. To compute a global covered area, this separator
 * must be projected over time using tools such as SepProj, or integrated into a
 * higher-level class dedicated to coverage computation.
 *
 * Using a time interval equal to the full trajectory domain directly with this
 * separator will generally lead to over-approximation, because tube evaluation
 * over a time interval loses temporal correlation.
 *
 * --------------------------------------------------------------------------
 * Relationship with SepDynDist
 * --------------------------------------------------------------------------
 * SepDynDisk is a specialization of SepDynDist for the case where the perception
 * region is a disk (i.e., inner radius equal to zero).
 *
 * In SepDynDist, the perception region is an annulus defined by:
 *   - an inner radius d_min
 *   - an outer radius d_max
 *
 * When d_min = 0, the annulus degenerates into a disk.
 *
 * SepDynDisk implements this specific case in a more efficient way than
 * SepDynDist by simplifying the separator structure and avoiding unnecessary
 * contractors.
 *
 * --------------------------------------------------------------------------
 * Optimization rationale
 * --------------------------------------------------------------------------
 * In the general annulus case (SepDynDist):
 *   - The inside set is defined by one distance contractor
 *   - The outside set is composed of two disjoint regions:
 *       * distances smaller than the inner radius
 *       * distances larger than the outer radius
 *   - This requires a CtcUnion to represent the outside set
 *
 * In the disk case (SepDynDisk):
 *   - There is no inner radius
 *   - The outside set is simply: distance > outer radius
 *   - Only ONE contractor is required for the outside set
 *
 * This avoids the use of CtcUnion and reduces the computational cost when the
 * separator is projected over time using SepProj.
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
 * When the separator is defined using a single interval `range`, perception is
 * modeled using a binary logic:
 *
 *   - Distance ∈ range        → the point is considered covered
 *   - Distance ∉ range        → the point is considered not covered
 *
 * This mode is well suited for simple coverage problems and for users who do not
 * need to explicitly model perception uncertainty.
 *
 * --------------------------------------------------------------------------
 * 2) Three-valued perception logic
 * --------------------------------------------------------------------------
 * When the separator is defined using both `range` and `uncertain_range`,
 * perception is modeled using a three-valued logic:
 *
 *   - Distance ∈ range
 *       → covered for sure
 *   - Distance ∈ (uncertain_range \ range)
 *       → unknown (may or may not have been perceived)
 *   - Distance ∉ uncertain_range
 *       → not covered for sure
 *
 * Although binary perception is mathematically a special case of three-valued
 * perception, these two modes are intentionally distinguished here to avoid
 * conceptual confusion for beginners.
 *
 * --------------------------------------------------------------------------
 * Geometric interpretation
 * --------------------------------------------------------------------------
 * At a given time t, the robot is located at p(t). The perception region is
 * defined as a disk centered at p(t) with radius d_max.
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
 * - Local separation of (x, y, t) boxes
 * - Projection over time using SepProj to compute covered areas
 * - Integration in paving algorithms for guaranteed coverage analysis
 *
 * This class is part of the codac-coverage-toolbox and is intended for
 * guaranteed, set-based coverage and perception analysis.
 */

#ifndef __SEP_DYN_DISK_H___
#define __SEP_DYN_DISK_H___

#include "codac.h"
#include "CtcDynDist.h"

using namespace codac;

/*
 * @class SepDynDisk
 * @brief Local separator modeling disk-shaped perception along a trajectory.
 *
 * SepDynDisk is a local separator that represents the perception of a static
 * 2D point by a mobile robot following a known trajectory, using a distance
 * constraint between the robot position and the point at a given time.
 *
 * This class is a specialized, optimized version of SepDynDist for the case
 * where the perception region is a disk (i.e., inner radius = 0). While
 * SepDynDist can handle both annulus and disk-shaped regions, SepDynDisk
 * simplifies the separator structure and reduces computational cost.
 *
 * --------------------------------------------------------------------------
 * Local separator
 * --------------------------------------------------------------------------
 * SepDynDisk acts on a 3-dimensional IntervalVector (x, y, t), where:
 *   - (x, y) is the unknown position of the object
 *   - t is the (possibly uncertain) detection time
 *
 * The separator only describes the perception constraint locally at a given
 * time or over a small interval. It does not compute the covered area over
 * the entire trajectory by itself. For global coverage, use SepProj or integrate
 * into a higher-level coverage algorithm.
 *
 * --------------------------------------------------------------------------
 * Optimization compared to SepDynDist
 * --------------------------------------------------------------------------
 * In SepDynDist:
 *   - The inside set is defined by a single distance contractor
 *   - The outside set requires two contractors and a CtcUnion, to account for
 *     distances smaller than the inner radius and larger than the outer radius
 *
 * In SepDynDisk:
 *   - There is no inner radius
 *   - The outside set is simply distances larger than the outer radius
 *   - Only one contractor is needed for the outside set
 *
 * This avoids the use of CtcUnion and reduces computation time when projecting
 * the separator using SepProj.
 *
 * --------------------------------------------------------------------------
 * Perception models
 * --------------------------------------------------------------------------
 * SepDynDisk supports two perception logics:
 *
 * 1) Binary perception logic:
 *      - Distance ≤ range → covered
 *      - Distance > range → not covered
 *
 * 2) Three-valued perception logic:
 *      - Distance ≤ range → covered for sure
 *      - Distance ∈ (uncertain_range \ range) → unknown
 *      - Distance > uncertain_range → not covered for sure
 *
 * Binary logic is a special case of three-valued logic but is explicitly
 * distinguished here for beginner clarity.
 *
 * --------------------------------------------------------------------------
 * Contractors
 * --------------------------------------------------------------------------
 * Internally, SepDynDisk uses SepCtcPair:
 *   - _ctcin: contractor for the inside (covered) region
 *   - _ctcout: contractor for the outside (not covered / uncertain) region
 *
 * For the disk case, the _ctcout contractor is simplified compared to SepDynDist
 * because no inner radius exists, avoiding the need for CtcUnion.
 *
 * --------------------------------------------------------------------------
 * Typical usage
 * --------------------------------------------------------------------------
 * - Local separation of (x, y, t) boxes
 * - Projection over time using SepProj to compute covered areas
 * - Integration in paving algorithms (e.g., SIVIA) for guaranteed perception analysis
 */

class SepDynDisk : public SepCtcPair
{
    public:

        /*
         * @brief Constructor for a binary perception SepDynDisk separator using a range.
         *
         * This constructor defines a disk-shaped perception region along a 2D trajectory,
         * using binary logic:
         *      - Points at a distance ≤ range are considered covered
         *      - Points at a distance > range are considered not covered
         *
         * This is a simplified version compared to SepDynDist because there is no inner radius:
         * the outside region only includes points beyond the disk radius, which avoids using
         * a CtcUnion and reduces computation time when projecting the separator.
         *
         * --------------------------------------------------------------------------
         * Geometric interpretation
         * --------------------------------------------------------------------------
         * At time t, the perception region is a disk of radius `range` centered at the
         * robot position p(t):
         *
         *     d = sqrt((x - px)^2 + (y - py)^2)
         *     covered: d ≤ range
         *     not covered: d > range
         *
         * where (px, py) = robot position at time t, and (x, y) = point variable.
         *
         * --------------------------------------------------------------------------
         * Parameters
         * --------------------------------------------------------------------------
         * @param traj A TubeVector representing the robot trajectory. Must have dimension ≥ 2; only the first two components are used.
         * @param range Maximum perception range (disk radius). Must be > 0.
         * @param overpass_warning Boolean flag to enable warnings if the trajectory dimension > 2.
         *
         * --------------------------------------------------------------------------
         * Internals
         * --------------------------------------------------------------------------
         * - _ctcin: contractor defining the inside (covered) region
         * - _ctcout: contractor defining the outside (not covered) region (simplified compared to SepDynDist)
         *
         * --------------------------------------------------------------------------
         * Notes
         * --------------------------------------------------------------------------
         * - This constructor implements binary coverage logic only.
         * - For three-valued perception logic, use the constructor with `range` and `border_width`.
         */
        SepDynDisk(const TubeVector& traj, const double range, bool overpass_warning = false);

        /*
         * @brief Constructor for a three-valued perception SepDynDisk separator using a range and border width.
         *
         * This constructor defines a disk-shaped perception region along a 2D trajectory,
         * using three-valued logic:
         *      - Points at a distance ≤ range are covered for sure
         *      - Points at a distance ∈ (range, range + border_width) are uncertain
         *      - Points at a distance > range + border_width are not covered
         *
         * Compared to SepDynDist, the outside contractor is simplified because there is no inner radius,
         * which avoids using CtcUnion and reduces computation time.
         *
         * --------------------------------------------------------------------------
         * Geometric interpretation
         * --------------------------------------------------------------------------
         * At time t, the perception region is a disk of radius `range` with an uncertain border:
         *
         *     d = sqrt((x - px)^2 + (y - py)^2)
         *     covered:        d ≤ range
         *     uncertain:      range < d ≤ range + border_width
         *     not covered:    d > range + border_width
         *
         * where (px, py) = robot position at time t, and (x, y) = point variable.
         *
         * --------------------------------------------------------------------------
         * Parameters
         * --------------------------------------------------------------------------
         * @param traj A TubeVector representing the robot trajectory. Must have dimension ≥ 2; only the first two components are used.
         * @param range Maximum perception range for sure coverage. Must be > 0.
         * @param border_width Width of the uncertain region. Must be ≥ 0.
         * @param overpass_warning Boolean flag to enable warnings if the trajectory dimension > 2 or border_width issues.
         *
         * --------------------------------------------------------------------------
         * Internals
         * --------------------------------------------------------------------------
         * - _ctcin: contractor defining the inside (covered) region (range + border)
         * - _ctcout: contractor defining the outside (uncertain/not covered) region (simplified compared to SepDynDist)
         *
         * --------------------------------------------------------------------------
         * Notes
         * --------------------------------------------------------------------------
         * - This constructor implements three-valued coverage logic (covered / uncertain / not covered).
         * - For a binary disk, use the constructor with `range` only.
         */
        SepDynDisk(const TubeVector& traj, const double range, const double border_width, bool overpass_warning = false);
        
    private:
        // Contractor defining the "covered" (inside) region.
        // Note: despite the name _ctcin, it is actually used for the inside perception region.
        CtcDynDist _ctcin;

        // Contractor defining the "outside / uncertain" region as a union of CtcDynDist contractors.
        // Note: despite the name _ctcout, it is actually used for the outside / uncertain perception region.
        CtcDynDist _ctcout;
};

# endif