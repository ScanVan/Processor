//============================================================================
// Name        : Queue.cpp
// Author      : Marcelo Kaihara
// Version     : 1.0
// Copyright   :
// Description : It has the functions to manipulate a thread safe queue.
//============================================================================

#ifndef QUEUEF_HPP_
#define QUEUEF_HPP_

#include <memory>
#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <thread>

namespace ScanVan {

template<typename T>
class thread_safe_queue_future{
	std::mutex m;
	std::condition_variable cv;
	std::queue<T> queue;
public:
	thread_safe_queue_future() {};
	thread_safe_queue_future(thread_safe_queue_future const & other_queue) {};

	void push(T& value) {
		std::lock_guard<std::mutex> lg{m};
		queue.push(std::move(value));
		cv.notify_one();
	}

	T wait_pop() {
		std::unique_lock<std::mutex> lg{m};
		cv.wait(lg, [this] {
			return !queue.empty();
		});
		T ref = std::move(queue.front());
		queue.pop();
		return std::move(ref);
	}

	bool empty()
	{
		std::lock_guard<std::mutex> lg{m};
		return queue.empty();
	}

	size_t size() {
		std::lock_guard<std::mutex> lg{m};
		return queue.size();
	}

	void wait_pop(T&ref) {
		std::unique_lock<std::mutex> lg { m };
		cv.wait(lg, [this] {
			return !queue.empty();
		});
		ref = queue.front();
		queue.pop();
	}

	bool pop(T& ref) {
		std::lock_guard<std::mutex> lg { m };
		if (queue.empty()) {
			return false;
		} else {
			ref = queue.front();
			queue.pop();
			return true;
		}
	}
};

} /* namespace ScanVan */

#endif /* QUEUE_HPP_ */
