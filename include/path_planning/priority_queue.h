#pragma once

#include <set>
#include <stdexcept>

// TODO(smarden): move this out of `path_planning`.

namespace path_planning
{

// A priority queue implementation.
template <typename T>
class PriorityQueue
{
  public:
    // If the element `t` is not in the queue, insert it.
    // If the element `t` is in the queue at a higher cost, remove the existing high cost instance and insert the new
    // low cost instance.
    // If the element `t` is in the queue at a lower cost, do nothing.
    //
    // Complexity O(N).
    //
    // Returns whether or not the element was inserted into the queue.
    bool upsert(T t);

    // Returns whether or not the queue is empty.
    bool empty() const
    {
        return elements_.empty();
    }

    // Returns the number of elements in the queue.
    typename std::set<T>::size_type size() const
    {
        return elements_.size();
    }

    // Returns the first element from the queue. Said element is removed from the queue.
    //
    // Complexity amortized O(1).
    //
    // Throws an exception if the queue is empty.
    T pop();

    // Returns the first element from the queue. Said element remains in the queue.
    //
    // Complexity amortized O(1).
    //
    // Throws an exception if the queue is empty.
    const T &peek() const;

  private:
    // Underlying container for storing the sorted elements.
    std::set<T> elements_;
};

template <typename T>
bool PriorityQueue<T>::upsert(T t)
{
    // Remove the element if it is already in the underlying set but at a higher cost.
    // TODO(smarden): this is linear time at the moment.
    for (auto it = std::begin(elements_); it != std::end(elements_); ++it)
    {
        if (*it == t)
        {
            if (*it < t)
            {
                // Already exists at a lower cost.
                return false;
            }
            else
            {
                // Already exists, but a higher cost. Remove it and we'll replace it with the lower cost.
                elements_.erase(it);
                break;
            }
        }
    }

    elements_.emplace(std::move(t));

    return true;
}

template <typename T>
T PriorityQueue<T>::pop()
{
    if (empty())
    {
        throw std::runtime_error("pop() called on empty PriotyQueue");
    }

    // Remove the element from the front of the underlying set.
    auto front = std::move(*std::begin(elements_));
    elements_.erase(std::begin(elements_));

    return front;
}

template <typename T>
const T &PriorityQueue<T>::peek() const
{
    if (empty())
    {
        throw std::runtime_error("peek() called on empty PriotyQueue");
    }

    return *std::begin(elements_);
}

} // namespace path_planning
