/*
 * @file SepDynPie.cpp
 * @brief Implementation of the SepDynPie local separator for dynamic pie-shaped perception.
 *
 * This file contains the implementation of the SepDynPie class, a separator derived
 * from codac::SepCtcPair. SepDynPie computes the perceived area for a vehicle equipped
 * with an oriented range sensor following a 2D trajectory.
 *
 * In contrast with SepDynPolar, this class is restricted to the case where the
 * minimum perception range is zero. The perception region therefore always has the
 * shape of a circular sector (pie).
 *
 * This restriction allows a simpler formulation of the outside region, which can be
 * computed using fewer contractors. This optimization is particularly beneficial when
 * SepDynPie is used inside higher-level operators such as SepProj for coverage
 * computation.
 *
 * The separator represents:
 *   - Inside the perception set (guaranteed perceived area) using _ctcin (CtcDynPolar)
 *   - Outside the perception set (guaranteed not perceived area) using _ctcout (CtcUnion)
 *
 * Key implementation notes:
 *   - The trajectory is assumed thin (without uncertainty)
 *   - Only the first three components of the trajectory are used
 *     (x position, y position, heading)
 *   - All angular quantities are expressed in the robot frame
 *
 * See SepDynPie.h for full class and geometric documentation.
 */

#include "SepDynPie.h"
#include "CtcDynDist.h"

namespace 
{
    // -------------------------
    // ---- Helpers ----
    // -------------------------

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
     * This mechanism is primarily a safety feature to make the class robust
     * to poorly conditioned angular inputs.
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

    // -------------------------
    // ---- _ctcin creation ----
    // -------------------------
    
    /*
     * @brief Verify inputs and create _ctcin for binary pie-shaped perception
     *        using a field-of-view angle and offset.
     *
     * The perceived region is defined as:
     *   - Distance ∈ [0, range]
     *   - Angle ∈ [fov_offset − fov/2 , fov_offset + fov/2] (robot frame)
     */
    CtcDynPolar verify_and_generate_ctcin_init(const TubeVector& traj, const double range, const double fov, const double fov_offset, bool overpass_warning)
    {
        // Verify trajectory dimension
        if (traj.size()<3)
        {
            throw std::range_error("[ERROR] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double fov, const double fov_offset, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
        }
        if (traj.size()>3 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double fov, const double fov_offset, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used" <<std::endl;
        }

        // Verify distance range
        if (range <= 0) 
        {
            throw std::range_error("[ERROR] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double fov, const double fov_offset, bool overpass_warning): The input double range must be greater than 0");
        }

        // Verify FOV angle
        double fov_angle = fov;
        if (fov_angle<0)
        {
            fov_angle = -1.*fov_angle;
        }
        if (fov_angle>=2*M_PI)
        {
            if (!overpass_warning)
            {
                std::cerr<<"[WARNING] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double fov, const double fov_offset, bool overpass_warning): You initialized an instance of class SepDynPie with an opening_angle superior or equal to 2*PI. Please use the SepDynDisk class for better performances."<<std::endl;
            }
            return CtcDynPolar(traj, Interval(0,range), (Interval(0)|Interval(Interval::TWO_PI)) - Interval::PI, true);
        }

        // Normalize angular offset into [-π, π]
        double fov_offset_angle = fov_offset;
        while (fov_offset_angle>M_PI)
        {
            fov_offset_angle = fov_offset_angle - 2*M_PI;
        }
        while (fov_offset_angle<-M_PI)
        {
            fov_offset_angle = fov_offset_angle + 2*M_PI;
        }

        // Create the polar contractor defining the perceived region
        return CtcDynPolar(traj, Interval(0,range), Interval(fov_offset_angle).inflate(fov_angle/2.), true);
    }

    /*
     * @brief Verify inputs and create _ctcin for binary pie-shaped perception
     *        using an explicit angular interval.
     */
    CtcDynPolar verify_and_generate_ctcin_init2(const TubeVector& traj, const double range, const Interval delta, bool overpass_warning)
    {
        // Verify trajectory dimension
        if (traj.size()<3)
        {
            throw std::range_error("[ERROR] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const Interval delta, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
        }
        if (traj.size()>3 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const Interval delta, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used" <<std::endl;
        }

        // Verify distance range
        if (range <= 0) 
        {
            throw std::range_error("[ERROR] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const Interval delta, bool overpass_warning): The input double range must be greater than 0");
        }

        // Verify delta angle
        Interval mod_delta = modulo2pi(delta);
        if (mod_delta.ub()-mod_delta.lb()>=2*M_PI)
        {
            if (!overpass_warning)
            {
                std::cerr<<"[WARNING] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const Interval delta, bool overpass_warning):You initialized an instance of class SepDynPie with an Interval delta with a width superior or equal to 2PI. Please use the SepDynDisk class for better performances"<<std::endl;
            }
            return CtcDynPolar(traj, Interval(0,range), (Interval(0)|Interval(Interval::TWO_PI)) - Interval::PI, true);
        }
        
        return CtcDynPolar(traj, Interval(0,range), mod_delta, true);
    }

    /*
     * @brief Verify inputs and create _ctcin for three-valued pie-shaped perception.
     *
     * The uncertain perception zone is modeled by extending both the range
     * and the angular interval.
     */
    CtcDynPolar verify_and_generate_ctcin_init3(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, bool overpass_warning)
    {
        // Verify trajectory dimension
        if (traj.size()<3)
        {
            throw std::range_error("[ERROR] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
        }
        if (traj.size()>3 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used" <<std::endl;
        }

        // Verify distance range
        if (range <= 0) 
        {
            throw std::range_error("[ERROR] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, bool overpass_warning): The input double range must be greater than 0");
        }

        // Verify range uncertain border
        if (range_border<0)
        {
            throw std::range_error("[ERROR] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, bool overpass_warning): The input double range_border must be greater than or equal to 0");
        }


        // Verify delta angle
        Interval mod_delta = modulo2pi(delta);
        if (mod_delta.ub()-mod_delta.lb()>=2*M_PI)
        {
            if (!overpass_warning)
            {
                std::cerr<<"[WARNING] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, bool overpass_warning):You initialized an instance of class SepDynPie with an Interval delta with a width superior or equal to 2PI. Please use the SepDynDisk class for better performances"<<std::endl;
            }
            mod_delta = (Interval(0)|Interval(Interval::TWO_PI)) - Interval::PI;
        }
        

        // Apply modulo 2pi to delta_border interval
        Interval mod_delta_border = modulo2pi(delta_border);
        
        // Check that delta is a subset of delta_borders
        if (!mod_delta.is_subset(mod_delta_border))
        {
            throw std::range_error("[ERROR] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, bool overpass_warning): The input Interval delta must be a subset of the Interval delta_border");
        }


        // Return the corresponding contractor
        return CtcDynPolar(traj, Interval(0,range+range_border), mod_delta_border, true);
    }

    // -------------------------
    // ---- _ctcout creation ----
    // -------------------------

    /*
     * @brief Generate _ctcout for binary pie-shaped perception
     *        using a field-of-view angle and offset.
     *
     * The outside region is defined as the union of:
     *   - Points located at a distance greater than range
     *   - Points located inside the range but outside the angular sector
     */
    CtcUnion generate_ctcout_init(const TubeVector& traj, const double range, const double fov, const double fov_offset, bool overpass_warning)
    {
        // Outside distance region
        CtcDynDist ctc_outside(traj, Interval(range, oo), true);

        // Normalize angular offset
        double fov_offset_angle = fov_offset;
        while (fov_offset_angle>M_PI)
        {
            fov_offset_angle = fov_offset_angle - 2*M_PI;
        }
        while (fov_offset_angle<-M_PI)
        {
            fov_offset_angle = fov_offset_angle + 2*M_PI;
        }

        double fov_angle = fov;
        if (fov_angle<0)
        {
            fov_angle = -1.*fov_angle; 
        }
        if (fov_angle>=2*M_PI)
        {
            // No angular exclusion inside the range
            CtcDynPolar ctc_complementary_inside(traj, Interval(0,range), Interval::EMPTY_SET, true);
            return CtcUnion(ctc_complementary_inside, ctc_outside);
        }

        // Complementary angular sector
        CtcDynPolar ctc_complementary_inside(traj, Interval(0,range), Interval(fov_offset_angle-M_PI).inflate((2*M_PI - fov_angle)/2.), true);
        return CtcUnion(ctc_complementary_inside, ctc_outside);
    }

    /*
     * @brief Generate _ctcout for binary pie-shaped perception
     *        using an explicit angular interval.
     */
    CtcUnion generate_ctcout_init2(const TubeVector& traj, const double range, const Interval delta, bool overpass_warning)
    {
        // Outside distance region
        CtcDynDist ctc_outside(traj, Interval(range, oo), true);

        if (delta.ub()-delta.lb()>=2*M_PI)
        {
            // No angular exclusion inside the range
            CtcDynPolar ctc_complementary_inside(traj, Interval(0,range), Interval::EMPTY_SET, true);
            return CtcUnion(ctc_complementary_inside, ctc_outside);
        }
        
        // Complementary angular sector
        Interval compl_delta = Interval(delta.ub(), delta.lb() + 2*M_PI);
        compl_delta = modulo2pi(compl_delta);

        CtcDynPolar ctc_complementary_inside(traj, Interval(0,range), compl_delta, true);
        
        return CtcUnion(ctc_complementary_inside, ctc_outside);
    }

    /*
     * @brief Generate _ctcout for three-valued pie-shaped perception.
     */
    CtcUnion generate_ctcout_init3(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, bool overpass_warning)
    {
        // Outside distance region
        CtcDynDist ctc_outside(traj, Interval(range, oo), true);

        if (delta.ub()-delta.lb()>=2*M_PI)
        {
            // No angular exclusion inside the range
            CtcDynPolar ctc_complementary_inside(traj, Interval(0,range), Interval::EMPTY_SET, true);
            return CtcUnion(ctc_complementary_inside, ctc_outside);
        }
        
        // Complementary angular sector
        Interval compl_delta = Interval(delta.ub(), delta.lb() + 2*M_PI);
        compl_delta = modulo2pi(compl_delta);

        CtcDynPolar ctc_complementary_inside(traj, Interval(0,range), compl_delta, true);
        
        return CtcUnion(ctc_complementary_inside, ctc_outside);
    }
}

// -------------------------
// ---- Constructors ----
// -------------------------

SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double fov, const double fov_offset, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init(traj, range, fov, fov_offset, overpass_warning)),
_ctcout(generate_ctcout_init(traj, range, fov, fov_offset, overpass_warning)),
SepCtcPair(_ctcout,_ctcin)
{}

SepDynPie::SepDynPie(const TubeVector& traj, const double range, const Interval delta, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init2(traj, range, delta, overpass_warning)),
_ctcout(generate_ctcout_init2(traj, range, delta, overpass_warning)),
SepCtcPair(_ctcout,_ctcin)
{}

SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init3(traj, range, range_border, delta, delta_border, overpass_warning)),
_ctcout(generate_ctcout_init3(traj, range, range_border, delta, delta_border, overpass_warning)),
SepCtcPair(_ctcout,_ctcin)
{}