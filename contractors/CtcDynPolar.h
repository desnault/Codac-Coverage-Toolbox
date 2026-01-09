/*
 * @file CtcDynPolar.h
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 *
 * @brief Definition of a dynamic polar contractor for oriented range sensors in CODAC.
 *
 * This file defines the class CtcDynPolar, a custom contractor derived from
 * codac::Ctc and adapted from the static CtcPolar contractor.
 *
 * The contractor enforces a polar constraint between a static 2D point and
 * a dynamic pose obtained from a time-parameterized trajectory represented
 * as a TubeVector.
 *
 * Given:
 *   - [x1], [x2] : Intervals representing the coordinates of a static 2D point
 *   - [p](.)     : TubeVector of dimension ≥ 3 representing a trajectory
 *                  ([p]([t]) = ([px], [py], [heading], ...))
 *   - [t]        : Interval representing the considered time
 *   - [range]    : Interval representing the admissible radial distance
 *   - [delta]    : Interval representing the angular field of view (FOV)
 *
 * the contractor enforces the polar constraints:
 *
 *   [x1] - [px] = [r] * cos([heading] + [delta])
 *   [x2] - [py] = [r] * sin([heading] + [delta])
 *
 * where:
 *   - ([px], [py], [theta]) = [p]([t]) are extracted from the trajectory,
 *   - [r] ∈ [range] is the admissible sensing range,
 *   - [delta] defines the sensor field of view relative to the robot heading.
 *
 * Notes:
 * - This contractor models an oriented range sensor with limited angular
 *   aperture (polar sector).
 * - The angular interval [delta] may be asymmetric to represent sensor offset
 *   relative to the robot heading.
 * - Only the first three components of the trajectory are used:
 *     ([x], [y], [heading]).
 * - The trajectory is assumed to be thin (without uncertainty) and is not
 *   contracted.
 * - The contractor performs contraction on:
 *     - Spatial variables: [x1], [x2]
 *     - Temporal variable: [t]
 *
 * This contractor is intended as a low-level building block and is typically
 * wrapped by higher-level separators (e.g., SepDynPolarProj) for coverage
 * computation and visualization.
 */

#ifndef __CTC_DYN_POLAR_H___
#define __CTC_DYN_POLAR_H___

#include "codac.h"

using namespace codac;

/*
 * @class CtcDynPolar
 * @brief Contractor enforcing a dynamic polar constraint with an oriented sensor.
 *
 * CtcDynPolar is a contractor adapted from codac::CtcPolar in which the polar
 * constraint is enforced between:
 *   - a static 2D point ([x1], [x2]), and
 *   - a dynamic pose ([px], [py], [heading]) obtained by evaluating a trajectory
 *     at a time interval [t].
 *
 * The enforced constraints correspond to a polar sector defined by:
 *   - a radial distance interval [range], and
 *   - an angular field of view [delta] relative to the robot heading.
 *
 * The trajectory [p](.) and the sensing parameters ([range], [delta]) are
 * provided at construction time and are assumed to remain constant.
 *
 * The trajectory is assumed to be thin (i.e., without uncertainty) and is
 * therefore not contracted by this contractor.
 *
 * During contraction, the contractor acts on:
 *   - the spatial variables [x1] and [x2], and
 *   - the temporal variable [t], through inversion of the trajectory.
 *
 * The trajectory must have a dimension greater than or equal to 3. Only the
 * first three components are interpreted as:
 *   - x-position,
 *   - y-position,
 *   - heading angle (theta, expressed in radians).
 *
 * This contractor addresses 2D problems but supports trajectories with higher
 * dimensions, provided that the first three components follow the above
 * convention.
 *
 * Users unfamiliar with polar constraints may refer to codac::CtcPolar for
 * a static formulation of the same constraint.
 */
class CtcDynPolar : public Ctc
{
    public:

        /*
         * @brief Construct a dynamic polar contractor for an oriented range sensor.
         *
         * This constructor initializes a contractor enforcing a polar constraint
         * between a static 2D point and a dynamic pose obtained from a trajectory.
         *
         * The sensing parameters ([range], [delta]) are fixed at construction time
         * and are assumed to remain constant along the trajectory.
         *
         * The trajectory is assumed to be thin (i.e., without uncertainty) and
         * is therefore not contracted.
         *
         * @param traj Time-parameterized trajectory represented as a TubeVector.
         *             The trajectory must have a dimension greater than or equal
         *             to 3. Only the first three components are used and interpreted
         *             as (x, y, heading).
         * @param range Interval representing the admissible radial distance.
         *              The interval must be a subset of [0, +∞).
         * @param delta Interval representing the angular field of view (FOV) of the
         *              sensor, expressed relative to the robot heading.
         *              The interval may be asymmetric.
         * @param overpass_warning Internal convenience flag allowing higher-level
         *                        classes to disable dimension-related warnings
         *                         issued by this contractor.
         *
         * @throws std::range_error if:
         *   - the trajectory dimension is less than 3,
         *   - the range interval is not non-negative.
         *
         * @warning If the width of delta is greater than or equal to 2π, the
         *          contraction on angles becomes unnecessary. In such cases,
         *          the omnidirectional contractor CtcDynDist is more efficient.
         */
        CtcDynPolar(const TubeVector &traj, const Interval range, const Interval delta, bool overpass_warning = false);
        
        /*
         * @brief Contract a static 2D point and time using the dynamic polar constraint.
         *
         * This method is the core contraction routine of CtcDynPolar.
         * It enforces the polar constraint between:
         *   - a static 2D point ([x1], [x2]), and
         *   - a dynamic pose ([px], [py], [theta]) obtained by evaluating the
         *     trajectory at time [t].
         *
         * The contraction is performed on:
         *   - the spatial variables [x1] and [x2], and
         *   - the temporal variable [t].
         *
         * The trajectory itself is assumed to be thin and is not contracted.
         *
         * @param x1 Interval representing the x-coordinate of the static point.
         * @param x2 Interval representing the y-coordinate of the static point.
         * @param t  Interval representing the time variable.
         *           It must be a subset of the trajectory time domain.
         *
         * @throws std::range_error if the time interval is not included in the
         *         trajectory time domain.
         */
        void contract(Interval &x1, Interval &x2, Interval &t);
        
        /*
         * @brief Convenience wrapper for contracting a 2D point stored in an IntervalVector.
         *
         * This method forwards the contraction to:
         *   contract(x[0], x[1], t)
         *
         * Only the first two components of the input vector are used.
         *
         * @param x IntervalVector containing the static point coordinates.
         *          Its dimension must be greater than or equal to 2.
         * @param t Interval representing the time variable.
         *
         * @throws std::range_error if:
         *   - the vector dimension is less than 2,
         *   - the time interval is not included in the trajectory time domain.
         */
        void contract(IntervalVector &x, Interval &t);
        
        /*
         * @brief Convenience wrapper for contracting a stacked vector (x1, x2, t).
         *
         * This method forwards the contraction to:
         *   contract(h[0], h[1], h[2])
         *
         * Only the first three components of the input vector are used.
         *
         * @param h IntervalVector containing the static point coordinates and time.
         *          The expected format is h = ([x1], [x2], [t]).
         *
         * @throws std::range_error if:
         *   - the vector dimension is less than 3,
         *   - the time interval is not included in the trajectory time domain.
         */
        void contract(IntervalVector &h);
    
    protected:
        /*
         * @brief Time-parameterized trajectory of the dynamical system.
         *
         * The trajectory is represented as a TubeVector of dimension ≥ 3.
         * Only the first three components are interpreted as:
         *   - x-position,
         *   - y-position,
         *   - heading angle.
         *
         * The trajectory is assumed to be thin (without uncertainty) and
         * is not contracted by this contractor.
         */
        const TubeVector& _traj;

        /*
         * @brief Admissible radial distance interval.
         *
         * This interval represents the fixed sensing range of the oriented
         * range sensor. It must be a subset of [0, +∞).
         */
        const Interval _range;

        /*
         * @brief Angular field of view (FOV) relative to the robot heading.
         *
         * This interval defines the admissible angular deviation from the
         * robot heading. It may be asymmetric and is internally normalized
         * to lie within [-π, π].
         */
        const Interval _delta;

        /*
         * @brief Internal flag controlling dimension-related warnings.
         *
         * This flag allows higher-level classes to disable warnings emitted
         * by this contractor when input dimensions exceed the expected ones.
         */
        const bool _overpass_warning;

};

# endif