#ifndef MACRO_H
#define MACRO_H

#include "Core/Platform.h"

namespace PhysIKA {
#define INVALID -1

/**
     * @brief Find the left bound of a target with a binary search algorithm
     * 
     * @tparam T 
     * @param target 
     * @param data Start address
     * @param maxSize size of array buffer
     * @return case 1: return INVALID if data is empty;
     *           case 2: return the index of the first element that is equal to or greater than target
     *           case 3: return maxSize if target is not in the array
     */
template <typename T>
COMM_FUNC int leftBound(T target, T* startLoc, size_t maxSize)
{
    int left  = 0;
    int right = maxSize;

    while (left < right)
    {
        int mid = (left + right) / 2;
        if (startLoc[mid] == target)
        {
            right = mid;
        }
        else if (startLoc[mid] < target)
        {
            left = mid + 1;
        }
        else if (startLoc[mid] > target)
        {
            right = mid;
        }
    }
    return left;
}

/**
     * @brief Find the right bound of a target with a binary search algorithm
     *
     * @tparam T
     * @param target
     * @param data Start address
     * @param maxSize size of array buffer
     * @return case 1: return -1 if data is empty;
     *           case 2: return the index of the last element that is equal to or smaller than target
     *           case 3: return -1 if target is not in the array
     */
template <typename T>
COMM_FUNC int rightBound(T target, T* startLoc, size_t maxSize)
{
    if (maxSize == 0)
        return -1;
    int left = 0, right = maxSize;

    while (left < right)
    {
        int mid = (left + right) / 2;
        if (startLoc[mid] == target)
        {
            left = mid + 1;
        }
        else if (startLoc[mid] < target)
        {
            left = mid + 1;
        }
        else if (startLoc[mid] > target)
        {
            right = mid;
        }
    }
    return left - 1;
}
}  // namespace PhysIKA
#endif  // MACRO_H
