/*
 * @file CtcDynDist.h
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 *
 * @brief Definition of a dynamic Euclidean distance contractor for CODAC.
 *
 * This file defines the class CtcDynDist, a custom contractor derived from
 * codac::Ctc and adapted from the existing CtcDist contractor.
 *
 * The contractor enforces an Euclidean distance constraint between a 2D
 * static point and a 2D position extracted from a time-parameterized trajectory
 * represented as a TubeVector.
 *
 * Given:
 *   - [x1], [x2] : Intervals representing the coordinates of a static 2D point
 *   - [p](.)   : TubeVector of dimension ≥ 2 representing a trajectory
 *                ([p]([t]) = ([px],[py], ...))
 *   - [t]      : Interval representing the considered time
 *   - [range]      : Interval representing the allowed Euclidean distance
 *
 * the contractor enforces the constraint:
 *
 *   [range] = sqrt( sqr([x1]-[px]) + sqr([x2]-[py]) )
 *
 * where ([px],[py]) = [p]([t]) are the first two components of the trajectory
 * at time [t].
 *
 * Notes:
 * - Only the first two components of the trajectory are used as the position
 *   ([px],[py]). If the TubeVector has more components, the user must ensure that
 *   the first two represent the spatial coordinates.
 * - The trajectory is assumed to be "thin" (without uncertainty), i.e., no
 *   contraction is performed on it.
 * - The contractor performs contraction on:
 *     - Spatial variables: [x1], [x2]
 *     - Temporal variable: [t] (time contraction)
 *
 * This contractor was initially designed for experiments in robotic coverage
 * estimation but can be used as a general-purpose contractor within CODAC.
*/

#ifndef __CTC_DYN_DIST_H___
#define __CTC_DYN_DIST_H___

#include "codac.h"

using namespace codac;

/*
 * @class CtcDynDist
 * @brief Contractor enforcing a dynamic Euclidean distance constraint.
 *
 * CtcDynDist is a contractor adapted from codac::CtcDist in which one of the
 * two static points involved in the Euclidean distance constraint is replaced
 * by a time-dependent position obtained from a trajectory.
 *
 * More precisely, the contractor enforces a Euclidean distance constraint
 * between:
 *   - a static 2D point ([x1], [x2]), represented by intervals, and
 *   - a dynamic 2D point ([px], [py]) = [p]([t]), obtained by evaluating a
 *     trajectory [p](.) at a time interval [t].
 *
 * The enforced constraint is:
 *
 *   [range] = sqrt( sqr([x1]-[px]) + sqr([x2]-[py]) )
 *
 * where ([px],[py]) = [p]([t]).
 *
 * The trajectory [p](.) and the distance interval [range] are provided at
 * construction time and are assumed to remain fixed. Variable or time-varying
 * distance intervals are not considered in this contractor.
 *
 * The trajectory [p](.) is assumed to be thin (i.e., without uncertainty).
 * Consequently, no contraction is performed on the trajectory itself.
 *
 * During contraction, the contractor acts on:
 *   - the spatial variables [x1] and [x2], and
 *   - the temporal variable [t], through inversion of the trajectory.
 *
 * The trajectory must have a dimension greater than or equal to 2. Only the
 * first two components are interpreted as the spatial coordinates ([x],[y]).
 * The user must ensure that the trajectory is formatted accordingly.
 *
 * Users unfamiliar with this formulation may refer to codac::CtcDist for a
 * simpler static Euclidean distance constraint.
 *
 * CtcDynDist can be used as a general-purpose contractor within the CODAC
 * framework, in particular for applications involving dynamical systems and
 * robotic trajectories.
*/
class CtcDynDist : public Ctc
{
    public:
        /*
         * @brief Construct a dynamic Euclidean distance contractor.
         *
         * This constructor initializes a contractor enforcing a Euclidean distance
         * constraint between a static 2D point and a dynamic point obtained from a
         * trajectory evaluated at a given time.
         *
         * The trajectory and the distance interval (range) are fixed at construction
         * time and are assumed to remain constant during the contractor lifetime.
         * The trajectory is assumed to be thin (i.e., without uncertainty) and is
         * therefore not contracted.
         *
         * @param traj Time-parameterized trajectory represented as a TubeVector.
         *             The trajectory must have a dimension greater than or equal to 2.
         *             Only the first two components are interpreted as the spatial
         *             coordinates (x,y).
         * @param range Interval representing the allowed Euclidean distance.
         *              The interval must be a subset of [0, +∞).
         * @param overpass_warning Internal convenience flag allowing higher-level
         *                         classes to disable dimension-related warnings
         *                         issued by this contractor.
         *
         * @throws std::range_error if:
         *   - the trajectory dimension is less than 2,
         *   - the distance interval is not non-negative.
        */
        CtcDynDist(const TubeVector &traj, const Interval &range, bool overpass_warning = false);

        /*
         * @brief Contract a static 2D point and time using the dynamic distance constraint.
         *
         * This method is the core contraction routine of CtcDynDist.
         * It enforces the Euclidean distance constraint between:
         *   - a static 2D point ([x1], [x2]), and
         *   - a dynamic 2D point obtained by evaluating the trajectory at time [t].
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
         * This method is a convenience interface that forwards the contraction to
         * contract(Interval&, Interval&, Interval&) using:
         *   - x[0] as x1,
         *   - x[1] as x2.
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
         * This method is a convenience interface that forwards the contraction to
         * contract(Interval&, Interval&, Interval&) using:
         *   - h[0] as x1,
         *   - h[1] as x2,
         *   - h[2] as t.
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
         * The trajectory is represented as a TubeVector of dimension ≥ 2.
         * Only the first two components are interpreted as the spatial
         * coordinates (x,y).
         *
         * The trajectory is assumed to be thin (without uncertainty) and
         * is not contracted by this contractor.
        */
        const TubeVector _traj;

        /*
         * @brief Allowed Euclidean distance interval.
         *
         * This interval represents the fixed range of admissible distances
         * between the static point and the trajectory position.
         * It must be a subset of [0, +∞).
        */
        const Interval _range;

        /*
         * @brief Internal flag controlling dimension-related warnings.
         *
         * This flag allows higher-level classes to disable warnings emitted
         * by this contractor when input dimensions exceed the expected ones.
         */
        const bool _overpass_warning;
};

# endif