#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <stop_token>

template <typename T>
class ThreadSafeQueue {
private:
    std::queue<T> data_queue;
    mutable std::mutex mut;
    std::condition_variable_any data_cond;

public:
    ThreadSafeQueue() = default;

    // Push an item into the queue
    void push(T new_value) {
        std::lock_guard lock(mut);
        data_queue.push(std::move(new_value));
        data_cond.notify_one(); // Wake up one waiting consumer
    }

    // Modern C++20 "Wait and Pop"
    // If a stop is requested (e.g., program shutting down), it returns nullopt.
    std::optional<T> wait_and_pop(std::stop_token stoken) {
        std::unique_lock lock(mut);
        
        // C++20 feature: wait() can now take a stop_token.
        // It will wake up if:
        // 1. Data is available (returns true)
        // 2. Stop is requested (returns false)
        bool success = data_cond.wait(lock, stoken, [this] { 
            return !data_queue.empty(); 
        });

        if (!success || data_queue.empty()) {
            return std::nullopt;
        }

        T res = std::move(data_queue.front());
        data_queue.pop();
        return res;
    }

    bool empty() const {
        std::lock_guard lock(mut);
        return data_queue.empty();
    }
};