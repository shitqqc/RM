#ifndef ARMOR_DETECTOR_THREAD_POOL_HPP
#define ARMOR_DETECTOR_THREAD_POOL_HPP

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include "yolo.hpp"

namespace rm_auto_aim
{
struct Frame
{
  int id;
  cv::Mat img;
  int64_t t;
  //std_msgs::msg::Header header;
  std::vector<rm_auto_aim::Armor> armors;

  Frame() = default;

  Frame(int frame_id, cv::Mat image, int64_t timestamp)
    : id(frame_id),
      img(std::move(image)),
      t(timestamp),
      armors()
  {}

  Frame(Frame&& other) noexcept = default;

  Frame(const Frame& other)
    : id(other.id),
      img(other.img.clone()),  // 深拷贝图像
      t(other.t),
      armors(other.armors)
  {}

  // ✅ 移动赋值运算符
  Frame& operator=(Frame&& other) noexcept = default;

  // ✅ 拷贝赋值运算符
  Frame& operator=(const Frame& other) {
    if (this != &other) {
      id = other.id;
      img = other.img.clone();  // 深拷贝图像
      t = other.t;
      armors = other.armors;
    }
    return *this;
  }

  // ✅ 工具方法：检查帧是否有效
  bool isValid() const {
    return !img.empty() && id >= 0;
  }

  // ✅ 工具方法：获取图像尺寸信息
  std::string getImageInfo() const {
    if (img.empty()) {
      return "Empty image";
    }
    return "Size: " + std::to_string(img.cols) + "x" + std::to_string(img.rows) + 
           ", Channels: " + std::to_string(img.channels());
  }
};

inline std::vector<std::unique_ptr<rm_auto_aim::YOLO>> create_yolov5s(
  const std::string & model_path, int number, double conf_threshold, const std::string & device)
{
  const int THROUGHPUT_MODE_ = 1;
  std::vector<std::unique_ptr<rm_auto_aim::YOLO>> yolov5s;
  for (int i = 0; i < number; i++) {
    // 使用make_unique创建实例
    auto yolo_instance = std::make_unique<rm_auto_aim::YOLO>(model_path, conf_threshold, device, THROUGHPUT_MODE_);
    
    // 添加调试信息验证实例独立性
    RCLCPP_INFO(rclcpp::get_logger("yolo_factory"),
               "Created YOLO instance %d with address: %p", i, yolo_instance.get());
    
    yolov5s.push_back(std::move(yolo_instance));
  }
  return yolov5s;
}

class OrderedQueue
{
public:
  OrderedQueue() : current_id_(1), shutdown_(false) {} 
  
  ~OrderedQueue()
  {
    shutdown();
  }

  void shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_ = true;
      main_queue_ = std::queue<rm_auto_aim::Frame>();
      buffer_.clear();
    }
    cond_var_.notify_all();  // 通知所有等待的线程
  }

  void enqueue(const Frame & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (shutdown_) return;
    if (item.id < current_id_) {
      RCLCPP_WARN(rclcpp::get_logger("ordered_queue"), "too small id!");
      return;
    }

    if (item.id == current_id_) {
      main_queue_.push(item);
      current_id_++;

      auto it = buffer_.find(current_id_);
      while (it != buffer_.end()) {
        main_queue_.push(it->second);
        buffer_.erase(it);
        current_id_++;
        it = buffer_.find(current_id_);
      }

      if (main_queue_.size() >= 1) {
        cond_var_.notify_one();
      }
    } else {
      buffer_[item.id] = item;
    }
  }

  rm_auto_aim::Frame dequeue()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    cond_var_.wait(lock, [this]() { 
      return shutdown_ || !main_queue_.empty(); 
    });

    if (shutdown_) {
      return Frame{};  // 返回空帧
    }

    rm_auto_aim::Frame item = std::move(main_queue_.front());
    main_queue_.pop();
    return item;
  }


  bool dequeue_timeout(rm_auto_aim::Frame & item, std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!cond_var_.wait_for(lock, timeout, [this]() { 
      return shutdown_ || !main_queue_.empty(); 
    })) {
      return false;  // 超时
    }

    if (shutdown_ || main_queue_.empty()) {
      return false;
    }

    item = std::move(main_queue_.front());
    main_queue_.pop();
    return true;
  }

  //非阻塞的检查方法
  bool has_frames_ready() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !main_queue_.empty();
  }

  bool try_dequeue(rm_auto_aim::Frame & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (main_queue_.empty()) {
      return false;
    }
    item = std::move(main_queue_.front()); 
    main_queue_.pop();
    return true;
  }

  size_t get_size() { 
    std::lock_guard<std::mutex> lock(mutex_);
    return main_queue_.size() + buffer_.size(); 
  }

  // ✅ 添加状态监控方法
  void print_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(rclcpp::get_logger("ordered_queue"), 
               "OrderedQueue Status: current_id=%d, main_queue=%zu, buffer=%zu", 
               current_id_, main_queue_.size(), buffer_.size());
  }

  int current_id_;
  
private:
  std::queue<Frame> main_queue_;
  std::unordered_map<int, Frame> buffer_;
  mutable std::mutex mutex_;
  std::condition_variable cond_var_;
  bool shutdown_;
};

class ThreadPool
{
public:
  ThreadPool(size_t num_threads) : stop(false)
  {
    for (size_t i = 0; i < num_threads; ++i) {
      workers.emplace_back([this] {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(queue_mutex);
            condition.wait(lock, [this] { return stop || !tasks.empty(); });
            if (stop && tasks.empty()) {
              return;
            }
            task = std::move(tasks.front());
            tasks.pop();
          }
          task();
        }
      });
    }
  }

  ~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      stop = true;
      tasks = std::queue<std::function<void()>>();
    }
    condition.notify_all();
    for (std::thread & worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  // 添加任务到任务队列
  template <class F>
  void enqueue(F && f)
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      if (stop) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
      }
      tasks.emplace(std::forward<F>(f));
    }
    condition.notify_one();
  }

private:
  std::vector<std::thread> workers;         // 工作线程
  std::queue<std::function<void()>> tasks;  // 任务队列
  std::mutex queue_mutex;                   // 任务队列互斥锁
  std::condition_variable condition;        // 条件变量，用于等待任务
  bool stop;                                // 是否停止线程池
};

}  // namespace tools

#endif  // TOOLS__THREAD_POOL_HPP
