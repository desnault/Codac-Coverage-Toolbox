/*
 * @file SepDynDist.h
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 * 
 * @brief Local separator for distance-based perception along a dynamic trajectory.
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
 * SepDynDist is a local separator. It does NOT compute the covered area over an
 * entire trajectory by itself. To compute a global covered area, this separator
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
 * defined as an annulus centered at p(t):
 *
 *   - Inner radius: d_min
 *   - Outer radius: d_max
 *
 * If d_min = 0, the annulus degenerates into a disk.
 *
 * The class SepDynDisk provides an optimized specialization for the disk case
 * (d_min = 0). SepDynDist can still be used in this case, but SepDynDisk is
 * recommended for better performance.
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

#ifndef __SEP_DYN_DIST_H___
#define __SEP_DYN_DIST_H___

#include "codac.h"
#include "CtcDynDist.h"

using namespace codac;

/*
 * @class SepDynDist
 * @brief Local separator modeling distance-based perception along a trajectory.
 *
 * SepDynDist is a local separator that represents the perception of a static
 * 2D point by a mobile robot following a known trajectory, using a distance
 * constraint between the robot position and the point at a given time.
 *
 * The separator acts on a 3-dimensional IntervalVector (x, y, t), where:
 *   - (x, y) is the unknown position of the object,
 *   - t is the (possibly uncertain) detection time.
 *
 * For a given time t, the robot is assumed to be located at p(t), and the
 * perception region is defined as an annulus (or disk) centered at p(t).
 *
 * --------------------------------------------------------------------------
 * Local separator
 * --------------------------------------------------------------------------
 * SepDynDist is a local separator: it only describes the perception constraint
 * at a given time t (or over a small time interval).
 *
 * It does NOT compute the covered area over the entire trajectory by itself.
 * To compute a global covered area, this separator must be projected over time
 * using tools such as SepProj, or integrated into a higher-level coverage class.
 *
 * Using SepDynDist directly with a large time interval generally leads to
 * over-approximation due to loss of temporal correlation in tube evaluation.
 *
 * --------------------------------------------------------------------------
 * Perception logic
 * --------------------------------------------------------------------------
 * SepDynDist supports two perception models, depending on how it is constructed:
 *
 * 1) Binary perception logic
 *    - Defined using a single distance interval `range`
 *    - Points whose distance to the robot lies inside `range` are considered
 *      covered
 *    - Points outside `range` are considered not covered
 *
 * 2) Three-valued perception logic
 *    - Defined using two distance intervals:
 *        * `range` (covered for sure)
 *        * `uncertain_range` (possibly covered)
 *    - Interpretation:
 *        * distance ∈ range → covered for sure
 *        * distance ∈ (uncertain_range \ range) → unknown
 *        * distance ∉ uncertain_range → not covered for sure
 *
 * Although binary perception is a special case of three-valued perception,
 * both modes are explicitly distinguished to avoid confusion for beginners.
 *
 * --------------------------------------------------------------------------
 * Geometric interpretation
 * --------------------------------------------------------------------------
 * At time t, the perception region is an annulus defined by:
 *   - inner radius d_min
 *   - outer radius d_max
 *
 * If d_min = 0, the annulus degenerates into a disk.
 *
 * The class SepDynDisk provides an optimized implementation for the disk case.
 * SepDynDist can still be used when d_min = 0, but SepDynDisk is recommended
 * for better performance.
 *
 * --------------------------------------------------------------------------
 * Assumptions
 * --------------------------------------------------------------------------
 * - The trajectory is assumed to be thin (no uncertainty on the robot position)
 * - No contraction is performed on the trajectory itself
 *
 * --------------------------------------------------------------------------
 * Typical usage
 * --------------------------------------------------------------------------
 * - Local separation of (x, y, t) boxes
 * - Projection over time using SepProj to compute covered areas
 * - Use within paving algorithms (e.g., SIVIA) for guaranteed perception analysis
*/
class SepDynDist : public SepCtcPair
{
    public:
        /*
        * @brief Constructs a SepDynDist separator using explicit inner and outer range.
        *
        * This constructor defines a range-based perception model along a known
        * trajectory, where the perception region at time t is an annulus centered
        * at the robot position p(t).
        *
        * The annulus is defined by two real bounds:
        *   - r_in  : inner range (minimum perception range)
        *   - r_out : outer range (maximum perception range)
        *
        * Any point whose distance to the robot lies in [r_in, r_out] is considered
        * covered for sure. Points with range outside this interval are considered
        * not covered for sure.
        *
        * This constructor implements a binary perception logic (covered / not covered),
        * without an explicit uncertain boundary.
        *
        * --------------------------------------------------------------------------
        * Geometric interpretation
        * --------------------------------------------------------------------------
        * For a given time t, the perception region is:
        *
        *   r_in ≤ d ≤ r_out,  with  d = sqrt( sqr(px-x) + sqr(py-y) )
        *
        * where (px, py) = p(t) is the robot position.  
        * If r_in = 0, the perception region becomes a disk of radius r_out.
        * In this case, the SepDynDisk class is recommended for better performance,
        * although SepDynDist remains valid.
        *
        * --------------------------------------------------------------------------
        * Separator structure
        * --------------------------------------------------------------------------
        * Internally, the separator is built from:
        *   - an "inside" contractor enforcing the range to lie within [r_in, r_out]
        *   - an "outside" contractor defined as the union of:
        *       * ranges in [0, r_in]
        *       * ranges in [r_out, +∞)
        *
        * Due to the internal conventions of SepCtcPair, the naming of the internal
        * contractors (_ctcin / _ctcout) may appear counterintuitive, but the semantic
        * behavior of the separator is correct.
        *
        * --------------------------------------------------------------------------
        * Expected variables
        * --------------------------------------------------------------------------
        * The separator acts implicitly on an IntervalVector of dimension 3:
        *
        *   [ x , y , t ]
        *
        * where:
        *   - (x, y) is the unknown object position,
        *   - t is the (possibly uncertain) detection time.
        *
        * The trajectory is assumed to be thin and is not contracted.
        *
        * --------------------------------------------------------------------------
        * Parameters
        * --------------------------------------------------------------------------
        * @param traj
        *   Thin TubeVector representing the robot trajectory. Must have dimension ≥ 2.
        *   Only the first two components (x, y) are used.
        *
        * @param r_in
        *   Inner perception range (must satisfy r_in ≥ 0).
        *
        * @param r_out
        *   Outer perception range (must satisfy r_out ≥ r_in).
        *
        * @param overpass_warning
        *   If set to true, disables warning messages related to:
        *     - trajectory dimension greater than 2
        *     - r_in equal to 0 (disk case)
        *
        * --------------------------------------------------------------------------
        * @throws std::range_error
        *   If r_in > r_out, r_in < 0, or if the trajectory dimension is invalid.
        */
        SepDynDist(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning = false);
        
        /*
        * @brief Constructor for a binary perception SepDynDist separator using an interval range.
        *
        * This constructor defines a binary perception area (covered / not covered) around a 2D trajectory.
        * The perception region is an annulus (or disk if the lower bound is zero) defined by the input interval.
        * Points at a distance d from the trajectory satisfy: 
        *      range.lb() <= d <= range.ub()
        * with d = sqrt(sqr(px - x) + sqr(py - y)), where (px, py) is a 2D point variable and (x, y) is the position on the trajectory.
        *
        * @note For a more detailed theoretical explanation on the geometric and mathematical interpretation,
        *       as well as the internal construction using CtcDynDist contractors, see the documentation of the first constructor:
        *       SepDynDist(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning).
        *
        * @param traj A TubeVector representing the robot trajectory. Must have dimension >= 2; only the first two components are used.
        * @param range Interval representing the perception region: [inner_radius, outer_radius]. Points inside this range are considered covered. Outside points are considered not covered.
        * @param overpass_warning Boolean flag to enable warnings if the trajectory dimension > 2 or range lower bound = 0.
        *
        * @note For beginner users: this constructor implements binary coverage logic only. 
        *       For three-valued logic (covered / uncertain / not covered), use the constructor with `range` and `uncertain_range`.
        * 
        * @note Internally, this constructor creates a CtcDynDist contractor for the inside set and a CtcUnion of two CtcDynDist contractors for the outside set.
        */
        SepDynDist(const TubeVector& traj, const Interval range, bool overpass_warning = false);

        /*
        * @brief Constructor for a three-valued perception SepDynDist separator using a range and border width.
        *
        * This constructor defines a perception region around a 2D trajectory using three-valued logic:
        *      - Points at a distance within `range` are considered certainly covered.
        *      - Points in the border zone (range extended by `border_width`) are uncertain.
        *      - Points outside the extended range are considered not covered.
        *
        * Geometric interpretation:
        *      Let range = [r_min, r_max] and border_width = b. Then:
        *          - d < r_min - b          : not covered
        *          - r_min - b <= d < r_min : uncertain
        *          - r_min <= d <= r_max    : covered
        *          - r_max < d <= r_max + b : uncertain
        *          - d > r_max + b          : not covered
        *      with d = sqrt(sqr(px - x) + sqr(py - y)), (px, py) the point variable, (x, y) the trajectory position.
        *
        * @note This constructor uses `range` and `border_width` to define the uncertain region. For an asymmetric or custom uncertain region, use the constructor with `range` and `uncertain_range`.
        *
        * @param traj A TubeVector representing the robot trajectory. Must have dimension >= 2; only the first two components are used.
        * @param range Interval representing the certain perception region: points inside this range are certainly covered.
        * @param border_width Width of the uncertain region outside and inside the range. Must be >= 0. If too large compared to range.lb(), the lower bound of the uncertain region will be truncated to 0.
        * @param overpass_warning Boolean flag to enable warnings if the trajectory dimension > 2, range.lb() = 0, or border_width issues.
        *
        * @note Internally, the constructor creates a CtcDynDist contractor for the extended range and a CtcUnion contractor for the complement to manage the three-valued logic.
        * 
        * @note For more detailed geometric and mathematical explanation, see the first constructor:
        *       SepDynDist(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning).
        */
        SepDynDist(const TubeVector& traj, const Interval range, const double border_width, bool overpass_warning = false);
        
        /*
        * @brief Constructor for a three-valued perception SepDynDist separator using a certain range and an uncertain range.
        *
        * This constructor defines a perception region around a 2D trajectory using three-valued logic:
        *      - Points within `range` are considered certainly covered.
        *      - Points within `uncertain_range` but outside `range` are uncertain.
        *      - Points outside `uncertain_range` are considered not covered.
        *
        * Geometric interpretation:
        *      Let range = [r_min, r_max] and uncertain_range = [u_min, u_max] with u_min <= r_min, r_max <= u_max. Then:
        *          - d < u_min              : not covered
        *          - u_min <= d < r_min     : uncertain
        *          - r_min <= d <= r_max    : covered
        *          - r_max < d <= u_max     : uncertain
        *          - d > u_max              : not covered
        *      with d = sqrt(sqr(px - x) + sqr(py - y)), (px, py) the point variable, (x, y) the trajectory position.
        *
        * @note This constructor allows asymmetric uncertain regions and is the recommended constructor for three-valued perception problems.
        * @note For the special case where r_min = 0 and only a disk-shaped coverage is needed, consider using the optimized SepDynDisk class.
        *
        * @param traj A TubeVector representing the robot trajectory. Must have dimension >= 2; only the first two components are used.
        * @param range Interval representing the certain perception region: points inside this range are certainly covered.
        * @param uncertain_range Interval representing the uncertain perception region: points inside this range but outside `range` are uncertain. Must contain `range`.
        * @param overpass_warning Boolean flag to enable warnings if the trajectory dimension > 2, range.lb() = 0, or range/uncertain_range inconsistencies.
        *
        * @note Internally, the constructor creates a CtcDynDist contractor for the `uncertain_range` and a CtcUnion contractor to define the outside region for three-valued logic computation.
        * @note This separator is local and can be used in conjunction with SepProj to compute the guaranteed covered area over the trajectory.
        */
        SepDynDist(const TubeVector& traj, const Interval range, const Interval uncertain_range, bool overpass_warning = false);
    
    private:
        // Contractor defining the "covered" (inside) region.
        // Note: despite the name _ctcin, it is actually used for the inside perception region.
        CtcDynDist _ctcin;

        // Contractor defining the "outside / uncertain" region as a union of CtcDynDist contractors.
        // Note: despite the name _ctcout, it is actually used for the outside / uncertain perception region.
        CtcUnion _ctcout;
};

# endif