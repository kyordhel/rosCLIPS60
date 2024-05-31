/* ** ***************************************************************
* synq_queue.h
*
* Author: Mauricio Matamoros
*
* Implements a therad-safe syncrhonous queue under the
* producer-consumer pattern
*
** ** **************************************************************/
/** @file sync_queue.h
 * Implementation of the sync_queue class:
 * a therad-safe syncrhonous queue
 */

#ifndef __SYNC_QUEUE_H__
#define __SYNC_QUEUE_H__
#pragma once

/** @cond */
#include <queue>
#include <mutex>
#include <chrono>
#include <condition_variable>
/** @endcond */


/**
 * Implements a therad-safe syncrhonous queue of type T
 */
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
	std::condition_variable_any _cv;
	/**
	 * Lock to protect the queue's integrity
	 */
	mutable std::timed_mutex _m;



// Disable copy constructor and assignment op.
private:
	sync_queue(sync_queue const& obj) = delete;
	sync_queue& operator=(sync_queue const&) = delete;

public:
	/**
	 * Creates a new instance of a thread-safe synchronous queue
	 */
	sync_queue(){}
	/**
	 * Default destructor
	 * @remark Does nothing
	 */
	~sync_queue(){}

	/**
	 * Retrieves an element from the synchronous queue
	 * @return The retrieved element
	 */
	virtual const T consume() {
		std::unique_lock<std::timed_mutex> ul(this->_m);  // Exclusive access to the queue

		// Wait until there is an element in the queue
		// We use a while because there can be spurious wake ups
		while( this->_q.empty() )
			this->_cv.wait(ul);                           // Wait for an element to be enqueued
		T obj = this->_q.front();                        // Retrieve object
		this->_q.pop();                                   // Pop the queue
		return obj;
	}

	/**
	 * Checks whether the queue is empty or not
	 * @return true if the queue is empty, false otherwise
	 */
	virtual bool empty() {
		std::lock_guard<std::timed_mutex> lock(this->_m); // Exclusive access to the queue
		return this->_q.empty();                          // Return value
	}

	/**
	 * Enqueues an element in the synchronous queue
	 * @param obj The element to enqueue
	 */
	virtual void produce(const T& obj) {
		std::lock_guard<std::timed_mutex> lock(this->_m); // Exclusive access to the queue
		this->_q.push(obj);                               // Enqueue the element
		this->_cv.notify_one();                           // Notifies consumers
	}

	/**
	 * Retrieves an element from the synchronous queue
	 * @param obj      The dequeued object
	 * @param timeout  The amount of time to wait for the elemnt in milliseconds
	 * @return         true if the element was successfully enqueued before the timeout. false otherwise.
	 */
	virtual bool timedConsume(T& obj, const std::chrono::milliseconds& timeout) {
		std::unique_lock<std::timed_mutex> ul(this->_m);  // Exclusive access to the queue

		// Wait until there is an element in the queue
		if( this->_q.empty() &&              // Lambda to capture spurious wake ups
			!this->_cv.wait_for(ul, timeout, [this](){ return !this->_q.empty(); }))
			return false;                                 // Or leave if the queue remains empty
		obj = this->_q.front();                           // Retrieve object
		this->_q.pop();                                   // Pop the queue
		return true;
	}

	/**
	 * Attempts to enqueue an element into the synchronous queue
	 * @param obj      The element to enqueue
	 * @param timeout  The amount of time to wait for the enqueue op to finish in milliseconds
	 * @return         true if the element was successfully enqueued before the timeout. false otherwise.
	 */
	// template< class Rep, class Period >
	virtual bool timedProduce(const T& obj, const std::chrono::milliseconds& timeout) {
		if( !this->_m.try_lock_for(timeout) )       // Exclusive access to the queue
			return false;
		this->_q.push(obj);                         // Enqueue the element
		this->_cv.notify_one();                     // Notifies consumers
		this->_m.unlock();
		return true;
	}
};

#endif // __SYNC_QUEUE_H__
