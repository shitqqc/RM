#ifndef THREAD_SAFE_QUEUE_HPP
#define THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <optional>
#include <shared_mutex>

namespace rm_auto_aim
{
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
  }



  bool try_pop(T& value)
  {
      std::unique_lock<std::mutex> lock(mutex_);
      
      if (queue_.empty()) {
          return false;  // 队列为空，立即返回
      }
      
      value = queue_.front();
      queue_.pop();
      return true;
  }

  std::optional<T> try_pop()
  {
      std::unique_lock<std::mutex> lock(mutex_);
      
      if (queue_.empty()) {
          return std::nullopt;  // 队列为空，立即返回
      }
      
      T value = std::move(queue_.front());
      queue_.pop();
      return std::make_optional(std::move(value));
  }

bool try_pop_if_more_than(size_t n, T& value)
{
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.size() <= n) {
        return false;  // 队列元素数量不足，立即返回
    }

    value = std::move(queue_.front());
    queue_.pop();
    return true;
}

  bool more_than(size_t n) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    //std::shared_lock<std::shared_mutex> lock(mutex__);
    return queue_.size() > n;
  }
  void push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.size() >= max_size_) {
      if (PopWhenFull) {
        queue_.pop();
      } else {
        full_handler_();
        return;
      }
    }

    queue_.push(value);
    not_empty_condition_.notify_all();
  }

  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
      return;
    }

    value = queue_.front();
    queue_.pop();
  }

  T pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    T value = std::move(queue_.front());
    queue_.pop();
    return (value);
  }


T pop_if_more_than(size_t n)
{
    std::unique_lock<std::mutex> lock(mutex_);
    
    not_empty_condition_.wait(lock, [this, n] { return queue_.size() > n; });
    
    // Pop and return the front element
    T value = std::move(queue_.front());
    queue_.pop();
    return value;
}
  T front()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    return queue_.front();
  }

  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();
  }

  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
    not_empty_condition_.notify_all();  // 如果其他线程正在等待队列不为空，这样可以唤醒它们
  }


private:
  std::queue<T> queue_;
  size_t max_size_;
  mutable std::mutex mutex_;
  //mutable std::shared_mutex mutex__;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP