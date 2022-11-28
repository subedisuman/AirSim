#ifndef DummySharedResource_hpp
#define DummySharedResource_hpp

#include <mutex>
#include <condition_variable>
#include <memory>
#include <iostream>

namespace msr
{
namespace airlib
{

// d-ekf typedefs
typedef Eigen::Matrix<float, 34, 1> Vector2NXf;
typedef Eigen::Matrix<float, 34, 34> Matrix2NXx2NXf;


class DekfSharedResource
{
public:

    struct Consensus
    {
        Vector2NXf u;
        Vector2NXf q;
        Matrix2NXx2NXf U;
        Matrix2NXx2NXf X;
    };
    struct MEkf
    {
        Vector2NXf x;
        Matrix2NXx2NXf P;
        Vector3r accel;
        Vector3r gyro;
    };

    DekfSharedResource()
    {
        _consensus_d1.reset(new Consensus());
        _consensus_d2.reset(new Consensus());
        _mekf_d1.reset(new MEkf());
        _mekf_d2.reset(new MEkf());
        _is_writen_once_consensus_d1 = false;
        _is_writen_once_consensus_d2 = false;
        _is_writen_once_mekf_d1 = false;
        _is_writen_once_mekf_d2 = false;
    }

    // read and write of consensus d1
    Consensus readDataConsensusD1()
    {
        std::unique_lock<std::mutex> lock(_mutex_consensus_d1);
        _cv_consensus_d1.wait(lock, [&]{return _is_writen_once_consensus_d1;});
        return *_consensus_d1;
    }
    void writeDataConsensusD1(Consensus data)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex_consensus_d1);
            *_consensus_d1 = data;
            _is_writen_once_consensus_d1 = true;
        }
        _cv_consensus_d1.notify_all();
    }
    
    // read and write of consensus d2
    Consensus readDataConsensusD2()
    {
        std::unique_lock<std::mutex> lock(_mutex_consensus_d2);
        _cv_consensus_d2.wait(lock, [&]{return _is_writen_once_consensus_d2;});
        return *_consensus_d2;
    }
    void writeDataConsensusD2(Consensus data)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex_consensus_d2);
            *_consensus_d2 = data;
            _is_writen_once_consensus_d2 = true;
        }
        _cv_consensus_d2.notify_all();
    }

    // read and write of mekf d1
    MEkf readDataMEkfD1()
    {
        std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< <<<<<< <<< << readDataMEkfD1 called" << std::endl;
        std::unique_lock<std::mutex> lock(_mutex_mekf_d1);
        _cv_mekf_d1.wait(lock, [&]{return _is_writen_once_mekf_d1;});
        std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< <<<<<< <<< << readDataMEkfD1 done" << std::endl;
        return *_mekf_d1;
    }
    void writeDataMEkfD1(MEkf data)
    {
        {
            std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< <<<<<< <<< << writeDataMEkfD1 called" << std::endl;
            std::lock_guard<std::mutex> lock(_mutex_mekf_d1);
            if(_mekf_d1 == nullptr) std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< <<<<<< <<< << _mdekf_d2 null in the shared res." << std::endl;

            *_mekf_d1 = data;
            std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< <<<<<< <<< << writeDataMEkfD1 done" << std::endl;
            _is_writen_once_mekf_d1 = true;
        }
        _cv_mekf_d1.notify_all();
    }

    // read and write of mekf d2
    MEkf readDataMEkfD2()
    {
        std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< <<<<<< <<< << readDataMEkfD2 called" << std::endl;
        std::unique_lock<std::mutex> lock(_mutex_mekf_d2);
        _cv_mekf_d2.wait(lock, [&]{return _is_writen_once_mekf_d2;});
        std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< <<<<<< <<< << readDataMEkfD2 done" << std::endl;
        return *_mekf_d2;
    }
    void writeDataMEkfD2(MEkf data)
    {
        {
            std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< <<<<<< <<< << writeDataMEkfD2 called" << std::endl;
            std::lock_guard<std::mutex> lock(_mutex_mekf_d2);
            if(_mekf_d2 == nullptr) std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< <<<<<< <<< << _mdekf_d2 null in the shared res." << std::endl;

            *_mekf_d2 = data;
            std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< <<<<<< <<< << writeDataMEkfD2 done" << std::endl;
            _is_writen_once_mekf_d2 = true;
        }
        _cv_mekf_d2.notify_all();
    }

private:
    std::unique_ptr<Consensus> _consensus_d1;
    std::unique_ptr<Consensus> _consensus_d2;
    std::unique_ptr<MEkf> _mekf_d1;
    std::unique_ptr<MEkf> _mekf_d2;
    std::mutex _mutex_consensus_d1;
    std::mutex _mutex_consensus_d2;
    std::mutex _mutex_mekf_d1;
    std::mutex _mutex_mekf_d2;
    std::condition_variable _cv_consensus_d1;
    std::condition_variable _cv_consensus_d2;
    std::condition_variable _cv_mekf_d1;
    std::condition_variable _cv_mekf_d2;

public:
    bool _is_writen_once_consensus_d1;
    bool _is_writen_once_consensus_d2;
    bool _is_writen_once_mekf_d1;
    bool _is_writen_once_mekf_d2;
};

}}

#endif
