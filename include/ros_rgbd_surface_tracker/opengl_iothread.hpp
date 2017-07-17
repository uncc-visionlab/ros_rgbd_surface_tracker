/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   opengl_iothread.hpp
 * Author: arwillis
 *
 * Created on July 16, 2017, 7:58 AM
 */

#ifndef OPENGL_IOTHREAD_HPP
#define OPENGL_IOTHREAD_HPP

#ifdef __cplusplus

#include <iostream>

#include <boost/atomic.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/thread/thread.hpp>

namespace spsc {

    class spsc_shared_data {
    public:
        int iterations;
        int producer_count;
        boost::atomic_int consumer_count;
        boost::lockfree::spsc_queue<int, boost::lockfree::capacity<1024> > spsc_queue;
        boost::atomic<bool> done;

        spsc_shared_data() : iterations(10000000),
        producer_count(0), consumer_count(0), done(false) {

        }
    };

    class IOThread_Producer {
        spsc_shared_data *spsc_shared_data_ptr;
    public:

        IOThread_Producer(spsc_shared_data& data) {
            spsc_shared_data_ptr = &data;
        }

        void producer(void) {
            for (int i = 0; i != spsc_shared_data_ptr->iterations; ++i) {
                int value = ++spsc_shared_data_ptr->producer_count;
                while (!spsc_shared_data_ptr->spsc_queue.push(value))
                    ;
            }
        }
    };

    class IOThread_Consumer {
        spsc_shared_data *spsc_shared_data_ptr;
    public:

        IOThread_Consumer(spsc_shared_data& data) {
            spsc_shared_data_ptr = &data;
        }

        void consumer(void) {
            int value;
            while (!spsc_shared_data_ptr->done) {
                while (spsc_shared_data_ptr->spsc_queue.pop(value))
                    ++spsc_shared_data_ptr->consumer_count;
            }

            while (spsc_shared_data_ptr->spsc_queue.pop(value))
                ++spsc_shared_data_ptr->consumer_count;
        }
    };

    class SingleThreaded_IOProducerConsumer {
    public:
        spsc_shared_data Data;
        IOThread_Producer producer;
        IOThread_Consumer consumer;

        SingleThreaded_IOProducerConsumer() : producer(Data), consumer(Data) {
            std::cout << "boost::lockfree::queue is ";
            if (!Data.spsc_queue.is_lock_free())
                std::cout << "not ";
            std::cout << "lockfree" << std::endl;
        }

        void start() {
            boost::thread producer_thread(boost::bind(&IOThread_Producer::producer, &producer));
            boost::thread consumer_thread(boost::bind(&IOThread_Consumer::consumer, &consumer));
            producer_thread.join(); // wait for producer to finish
            Data.done = true;
            consumer_thread.join(); // wait for the consumer to finish
            std::cout << "produced " << Data.producer_count << " objects." << std::endl;
            std::cout << "consumed " << Data.consumer_count << " objects." << std::endl;
        }
    };
}
#endif /* __cplusplus */
#endif /* OPENGL_IOTHREAD_HPP */

