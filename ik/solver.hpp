#pragma once

#include "ik/ecs.hpp"

namespace ik
{

template <unsigned D>
class solver
{
public:
    solver() {}
    solver& setKeepTrace(bool keepTrace) { this->keepTrace = keepTrace; return *this; }
    solver& setMaxIterations(unsigned maxIterations) { this->maxIterations = maxIterations; return *this; }
    solver& setTolerance(float tolerance) { this->tolerance = tolerance; return *this; }

    ecs<D>* getTrace(unsigned time) { return trace[time]; }

protected:
    bool keepTrace;
    unsigned maxIterations;
    float tolerance;

    std::vector<ecs<D>*> trace;
};



} // namespace ik
