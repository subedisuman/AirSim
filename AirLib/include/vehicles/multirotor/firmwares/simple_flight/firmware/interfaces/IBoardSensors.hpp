#pragma once

namespace simple_flight
{

class IBoardSensors
{
public:
    virtual void readAccel(float accel[3]) const = 0; //accel in m/s^2
    virtual void readGyro(float gyro[3]) const = 0; //angular velocity vector rad/sec

    virtual bool checkImuIfNew() const = 0;
    virtual bool checkBarometerIfNew() const = 0;
    virtual bool checkMagnetometerIfNew() const = 0;
    virtual bool checkGpsIfNew() const = 0;
    virtual bool checkPODResultsIfNew() const = 0;

    virtual void readImuData(float accel[3], float gyro[3]) const = 0;
    virtual void readBarometerData(float* altitude) const = 0;
    virtual void readMagnetometerData(float mag[3]) const = 0;
    virtual void readGpsData(double geo[3], float vel[3]) const = 0;

    virtual void setPODResults(const std::vector<float>& lp_center_val, const std::vector<float>& lp_center_var_val, const std::vector<bool>& gps_denied) = 0;
    virtual void readPODResultsAndReset(float lp_center[2], float lp_center_var[2], bool gps_denied[1]) = 0;
    virtual bool checkIfGpsDenied() const = 0;

    virtual ~IBoardSensors() = default;
};
}
