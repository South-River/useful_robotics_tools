#ifndef _TIC_TOC_HPP
#define _TIC_TOC_HPP

#include <iostream>
#include <chrono>

namespace TicToc{
typedef double float64_t;
typedef float  float32_t;

enum{MILLI, MICRO, NANO};

class TicToc
{
private:
    uint8_t _status;
    uint8_t _flag = false;
    uint16_t _times = 0;
    std::chrono::_V2::steady_clock::time_point start, end;
public:
    TicToc(){_status = MILLI;};
    TicToc(const uint8_t& status):_status(status){}
    ~TicToc(){};
public:
    void tic()
    {
        start = std::chrono::steady_clock::now();
        _flag = true;
    }
    void tocAndPrint()
    {
        if (_flag == false)
        {
            printf("Please first call tic()\n");
            return;
        }
        end = std::chrono::steady_clock::now();
        if (_status == MILLI)
        {
            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
            printf("Time Cost (%d): %ld ms\n", _times, dt);
        }
        else if (_status == MICRO)
        {
            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
            printf("Time Cost (%d): %ld us\n", _times, dt);
        }
        else if (_status == NANO)
        {
            auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count();
            printf("Time Cost (%d): %ld ns\n", _times, dt);
        }
        ++_times;
        _flag = false;
    }

    float64_t toc()
    {
        if (_flag == false)
        {
            std::cout << "Please first call tic()" << std::endl;
            return -1.0;
        }
        end = std::chrono::steady_clock::now();
        if (_status == MILLI)
        {
            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
            return dt;
        }
        else if (_status == MICRO)
        {
            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
            return dt;
        }
        else if (_status == NANO)
        {
            auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count();
            return dt;
        }
        ++_times;
        _flag = false;
    }
};
};

#endif