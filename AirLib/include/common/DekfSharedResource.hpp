#ifndef DummySharedResource_hpp
#define DummySharedResource_hpp

#include <mutex>
#include <memory>

class DekfSharedResource
{
public:
    struct Data
    {
        float q = 0.0;
        float x = 0.0;
    };

    DekfSharedResource()
    {
        _data_d1.reset(new Data());
        _data_d2.reset(new Data());
    }

    void writeDataD1(float data)
    {
        const std::lock_guard<std::mutex> lock(_mutex);
        _data_d1->q = data;
    }

    void writeDataD2(float data)
    {
        const std::lock_guard<std::mutex> lock(_mutex);
        _data_d2->q = data;
    }

    float readDataD1()
    {
        const std::lock_guard<std::mutex> lock(_mutex);
        return _data_d1->q;
    }

    float readDataD2()
    {
        const std::lock_guard<std::mutex> lock(_mutex);
        return _data_d2->q;
    }

private:
    std::unique_ptr<Data> _data_d1;
    std::unique_ptr<Data> _data_d2;
    std::mutex _mutex;
    // std::mutex _mutex_d2;

};

#endif
