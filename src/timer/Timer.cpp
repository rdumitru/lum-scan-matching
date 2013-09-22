//==============================================================================
// Includes.
//==============================================================================
// User includes.
#include <timer/Timer.h>

// C++ includes.
#include <iostream>
using namespace std;

// C includes.
#include <assert.h>

//==============================================================================
// Class implementation.
//==============================================================================
Timer::Timer() {
    this->reset();
}

Timer::Timer(const Timer &other) {
    this->m_CpuTimes = other.m_CpuTimes;;
}

Timer::~Timer()
{}

void Timer::start() {
    this->reset();
    this->record();
}

void Timer::reset() {
    this->m_CpuTimes.clear();
    this->m_RealTimes.clear();
}

void Timer::record() {
    // Record the times.
    clock_t cpuTime = clock();

    timeval realTime;
    gettimeofday(&realTime, 0);

    // Put times in queue.
    this->m_CpuTimes.push_back(cpuTime);
    this->m_RealTimes.push_back(realTime);
}

void Timer::printTime(const string &msg,
                      const bool &sinceStart)
{
    cerr << msg << ": " << endl;
    cerr << "\t" << "CPU\t" << this->getCpuTime(sinceStart) << endl;
    cerr << "\t" << "REAL\t" << this->getRealTime(sinceStart) << endl;
}

long Timer::getCpuTime(const bool &sinceStart) {
    assert(this->m_CpuTimes.size() >= 2);

    // Get the second last recorded time.
    clock_t ref = *(this->m_CpuTimes.rbegin() + 1);
    clock_t last = this->m_CpuTimes.back();

    if (sinceStart == true) {
        ref = this->m_CpuTimes.front();
    }

    // Return in miliseconds.
    return (last - ref) * 1000 / CLOCKS_PER_SEC;
}

long Timer::getRealTime(const bool &sinceStart) {
    assert(this->m_RealTimes.size() >= 2);

    // Get the second last recorded time.
    timeval ref = *(this->m_RealTimes.rbegin() + 1);
    timeval last = this->m_RealTimes.back();

    if (sinceStart == true) {
        ref = this->m_RealTimes.front();
    }

    // Return in miliseconds.
    long secs = last.tv_sec - ref.tv_sec;
    long usecs = last.tv_usec - ref.tv_usec;

    return (long)(((secs * 1000) + (usecs / 1000.0)) + 0.5);
}
