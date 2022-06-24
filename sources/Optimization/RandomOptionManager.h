#include "vector"

#include "sources/Utils/Parameters.h"

/**
 * @brief The main responsibility of each random manager is:
 *  - Decide whether the current option is useful for the overall procedure
 *  - If current option is not useful, then select a different option to try
 *
 * However, considering that different manager requires different types of interfaces, we don't require real implementation of manger to inherit this class.
 * @tparam T option type
 */
template <typename T>
class RandomManagerBase
{
private:
    int currentOption_;
    std::vector<T> options_;

public:
    RandomManagerBase() {}
    virtual T GetCurrentOption();
    virtual bool IfCurrentOptionUseful();
    virtual void SwitchToNextOption();

    virtual void EvaluateCurrentOption(); // this function first exams whether current option is useful; if not, then it switchs to the next option
};

// ********************************************************************* //

class RandomManagerWeightLS
{
private:
    int currentOption_;
    std::vector<int> options_;

public:
    RandomManagerWeightLS(int maxOption = 10)
    {
        currentOption_ = 0;
        options_.reserve(maxOption);
        for (int i = 0; i < maxOption; i++)
        {
            options_.push_back(i);
        }
    }

    int inline GetCurrentOption() { return options_[currentOption_]; }

    bool IfCurrentOptionUseful(double prevError, double currError)
    {
        return (prevError - currError) / prevError > relativeErrorTolerance; // observable error decrease
    }
    inline void SwitchToNextOption() { currentOption_++; }
};

// ********************************************************************* //
