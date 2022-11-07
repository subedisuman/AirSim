#ifndef DummySharedResource_hpp
#define DummySharedResource_hpp

#include <mutex>

class DekfSharedResource
{
public:
    struct Data
    {
        struct concensus
        {
            int q;
        };

        struct micro_ekf
        {
            int x;
        };

        Data()
        {
        }
    };

    DekfSharedResource(){}

    void writeDataD1()
    {
        const std::lock_guard<std::mutex> lock(_mutex_d1);

    }
    void writeDataD2()
    {
        const std::lock_guard<std::mutex> lock(_mutex_d2);

    }
    void readDataD1()
    {
        const std::lock_guard<std::mutex> lock(_mutex_d1);

    }
    void readDataD2()
    {
        const std::lock_guard<std::mutex> lock(_mutex_d2);

    }

private:
    Data _data_d1;
    Data _data_d2;
    std::mutex _mutex_d1;
    std::mutex _mutex_d2;

};

#endif
