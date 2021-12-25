// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Gps_hpp
#define msr_airlib_Gps_hpp

#include <random>
#include "common/Common.hpp"
#include "GpsSimpleParams.hpp"
#include "GpsBase.hpp"
#include "common/FirstOrderFilter.hpp"
#include "common/FrequencyLimiter.hpp"
#include "common/DelayLine.hpp"

namespace msr
{
namespace airlib
{

    class GpsSimple : public GpsBase
    {
    public: //methods
        GpsSimple(const AirSimSettings::GpsSetting& setting = AirSimSettings::GpsSetting())
            : GpsBase(setting.sensor_name)
        {
            // initialize params
            params_.initializeFromSettings(setting);

            //initialize frequency limiter
            freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
            delay_line_.initialize(params_.update_latency);

            //initialize filters
            eph_filter.initialize(params_.eph_time_constant, params_.eph_final, params_.eph_initial); //starting dilution set to 100 which we will reduce over time to targeted 0.3f, with 45% accuracy within 100 updates, each update occurring at 0.2s interval
            epv_filter.initialize(params_.epv_time_constant, params_.epv_final, params_.epv_initial);
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            // added
            gauss_dist_pos.reset();
            gauss_dist_vel.reset();

            freq_limiter_.reset();
            delay_line_.reset();

            eph_filter.reset();
            epv_filter.reset();

            addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput());
        }

        virtual void update() override
        {
            GpsBase::update();

            freq_limiter_.update();
            eph_filter.update();
            epv_filter.update();

            if (freq_limiter_.isWaitComplete()) { //update output
                addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput());
            }

            delay_line_.update();

            if (freq_limiter_.isWaitComplete()){
                setOutput(delay_line_.getOutput());

                // added by Suman, isNew flag is set to true if the sensor signal updates
                is_new_ = true;
            }
        }

        //*** End: UpdatableState implementation ***//

        virtual ~GpsSimple() = default;

    private:
        void addOutputToDelayLine(real_T eph, real_T epv)
        {
            Output output;
            const GroundTruth& ground_truth = getGroundTruth();

            //GNSS
            output.gnss.time_utc = static_cast<uint64_t>(clock()->nowNanos() / 1.0E3);
            output.gnss.geo_point = ground_truth.environment->getState().geo_point;

            // // added by Suman
            // output.gnss.geo_point.longitude = ground_truth.kinematics->pose.position[0];
            // output.gnss.geo_point.latitude = ground_truth.kinematics->pose.position[1];
            // output.gnss.geo_point.altitude = ground_truth.kinematics->pose.position[2];

            real_T gps_sigma_pos = 0.00001f; // 1/6378km rad 
            output.gnss.geo_point.longitude += gauss_dist_pos.next()[0] * gps_sigma_pos;
            output.gnss.geo_point.latitude  += gauss_dist_pos.next()[1] * gps_sigma_pos;
            output.gnss.geo_point.altitude  += gauss_dist_pos.next()[2] * 3.0f;

            // //

            output.gnss.eph = eph;
            output.gnss.epv = epv;

            output.gnss.velocity = ground_truth.kinematics->twist.linear;
            output.gnss.velocity.x() += gauss_dist_vel.next()[0]*2.0f;
            output.gnss.velocity.y() += gauss_dist_vel.next()[1]*2.0f;
            output.gnss.velocity.z() += gauss_dist_vel.next()[2]*2.0f;

            output.is_valid = true;

            output.gnss.fix_type =
                output.gnss.eph <= params_.eph_min_3d   ? GnssFixType::GNSS_FIX_3D_FIX
                : output.gnss.eph <= params_.eph_min_2d ? GnssFixType::GNSS_FIX_2D_FIX
                                                        : GnssFixType::GNSS_FIX_NO_FIX;

            output.time_stamp = clock()->nowNanos();

            delay_line_.push_back(output);
        }

    private:
        typedef std::normal_distribution<> NormalDistribution;

        GpsSimpleParams params_;

        FirstOrderFilter<real_T> eph_filter, epv_filter;
        FrequencyLimiter freq_limiter_;
        DelayLine<Output> delay_line_;

        //added
        RandomVectorGaussianR gauss_dist_pos = RandomVectorGaussianR(0, 1);
        RandomVectorGaussianR gauss_dist_vel = RandomVectorGaussianR(0, 1);
    };
}
} //namespace
#endif
