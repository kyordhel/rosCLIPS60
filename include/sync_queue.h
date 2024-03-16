#ifndef __SYNC_QUEUE_H__
#define __SYNC_QUEUE_H__

#include <queue>
#include <mutex>
#include <condition_variable>

template <class T>
class sync_queue
{
private:
	/**
	 * The queue
	 */
	std::queue<T> _q;
	/**
	 * Synchronous counter for the number of objects in the queue
	 */
	std::condition_variable _cv;
	/**
	 * Lock to protect the queue's integrity
	 */
	mutable std::mutex _m;



// Disable copy constructor and assignment op.
private:
	sync_queue(sync_queue const& obj);
	sync_queue& operator=(sync_queue const&);

public:
	sync_queue() : _q(), _m(), _cv(){}
	~sync_queue(){}

	/**
	 * Retrieves an element from the synchronous queue
	 * @return The retrieved element
	 */
	virtual T& consume() {
		std::unique_lock<std::mutex> lock(this->_m); // Exclusive access to the queue

		// Wait until there is an element in the queue
		// We use a while because there can be spurious wake ups
		while( this->_q.empty() )
			this->_cv.wait(lock);   // Wait for an element to be enqueued
		T& obj = this->_q.front();   // Retrieve object
		this->_q.pop();             // Pop the queue
		return obj;
	}

	/**
	 * Checks whether the queue is empty
	 * @return true if the queue is empty, false otherwise
	 */
	virtual bool empty() {
		std::lock_guard<std::mutex> lock(this->_m); // Exclusive access to the queue
		return this->_q.empty();                    // Return value
	}

	/**
	 * Enqueues an element in the synchronous queue
	 * @param obj The element to enqueue
	 */
	virtual void produce(const T& obj) {
		std::lock_guard<std::mutex> lock(this->_m); // Exclusive access to the queue
		this->_q.push(obj);                         // Enqueue the element
		this->_cv.notify_one();                     // Notifies consumers
	}
};

#endif // __SYNC_QUEUE_H__
