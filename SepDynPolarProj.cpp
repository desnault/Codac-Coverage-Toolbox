/*
 * @file SepDynPolarProj.cpp
 * @brief Implementation of SepDynPolarProj, a high-level class for projecting
 *        the coverage of a moving vehicle equipped with an oriented range sensor.
 *
 * This class combines the SepDynPolar separator with the SepProj projection, 
 * making it easy for beginners to compute the polar (sector) coverage along a trajectory.
 *
 * See SepDynPolarProj.h for full class and constructor documentation.
 */

#include "SepDynPolarProj.h"

namespace 
{
    // -------------------------------
    // Helper function to safely wrap angles
    // -------------------------------

    /*
     * @brief Ensure an interval angle is expressed in [-π, π] range
     *
     * This function is used as a safety mechanism for beginners.
     * Users might define delta intervals exceeding 2π or outside [-π, π].
     *
     * - If delta width ≥ 2π, returns [-π, π] as full coverage.
     * - Otherwise, it shifts the interval to be centered within [-π, π].
     *
     * @param delta Input angular interval (in radians, robot frame)
     * @return Interval adjusted modulo 2π
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

    // -------------------------------
    // Verify input and generate SepDynPolar separator
    // -------------------------------

    /*
     * @brief Generate a binary coverage separator (inside/outside)
     */
    SepDynPolar verify_and_generate_sep(const TubeVector& traj, const Interval range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning)
    {
        // Trajectory dimension must be at least 3: (x, y, heading)
        if (traj.size()<3)
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
        }
        if (traj.size()>3 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used" <<std::endl;
        }

        // Check range validity
        if (!range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
        }
        if(range.lb()==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): You initialized an instance of class SepDynPolarProj with the lower bound of the range being equal to 0 => Please use the SepDynPieProj class for better performances" <<std::endl;
        }

        // Check delta and wrap to [-π, π]
        Interval mod_delta = modulo2pi(delta);
        if (delta.ub()-delta.lb()>=2*M_PI && !overpass_warning)
        {
            std::cerr<<"[WARNING] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): You initialized an instance of class SepDynPolarProj with an Interval delta with a width greater than or equal to 2*PI. Please use the SepDynDistProj class for better performances."<<std::endl;
        }

        // Check projection time interval
        if (!t_proj.is_subset(traj.tdomain()))
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval t_proj must be a subset of the trajectory time domain");
        }

        // Check projection precision
        if (eps_proj<=0) 
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double eps_proj must be greater than 0");
        }

        // Create and return the separator
        return SepDynPolar(traj, range, mod_delta, true);
    }

    /*
     * @brief Generate a three-valued coverage separator (inside/uncertain/outside)
     */
    SepDynPolar verify_and_generate_sep2(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning)
    {
        // Trajectory dimension check
        if (traj.size()<3)
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
        }
        if (traj.size()>3 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used" <<std::endl;
        }

        // Range and uncertain_range checks
        if (!range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
        }
        if (range.lb()==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning): You initialized an instance of class SepDynPolarProj with the lower bound of the range being equal to 0 => Please use the SepDynPieProj class for better performances" <<std::endl;
        }

        if (!uncertain_range.is_subset(Interval(0,oo)))
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval uncertain_range must be a subset of [0,+inf]");
        }
        if (!range.is_subset(uncertain_range))
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval range must be a subset of the Interval uncertain_range");
        }

        // Delta and uncertain_delta checks
        Interval mod_delta = modulo2pi(delta);
        if (delta.ub()-delta.lb()>=2*M_PI && !overpass_warning)
        {
            std::cerr<<"[WARNING] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning): You initialized an instance of class SepDynPolarProj with an Interval delta with a width greater than or equal to 2*PI. Please use the SepDynDistProj class for better performances."<<std::endl;
        }

        Interval mod_uncertain_delta = modulo2pi(uncertain_delta);
        if (!mod_delta.is_subset(mod_uncertain_delta))
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval delta must be a subset of the Interval uncertain_delta");
        }

        // Projection time check
        if (!t_proj.is_subset(traj.tdomain()))
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval t_proj must be a subset of the trajectory time domain");
        }

        // Projection precision
        if (eps_proj<=0) 
        {
            throw std::range_error("[ERROR] SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double eps_proj must be greater than 0");
        }

        // Create and return three-valued separator
        return SepDynPolar(traj, range, uncertain_range, mod_delta, mod_uncertain_delta, true);
    }
}

// -------------------------------
// Constructor definitions
// -------------------------------

// Binary coverage
SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning):
_sep(verify_and_generate_sep(traj, range, delta, t_proj, eps_proj, overpass_warning)),
SepProj(_sep, t_proj, eps_proj) 
{}

// Three-valued coverage
SepDynPolarProj::SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning):
_sep(verify_and_generate_sep2(traj, range, uncertain_range, delta, uncertain_delta, t_proj, eps_proj, overpass_warning)), 
SepProj(_sep, t_proj, eps_proj) 
{}