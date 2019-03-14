//
// Created by lancern on 19-3-13.
//

#ifndef IMAGEQUEUE_IMAGEQUEUEIMPL_H
#define IMAGEQUEUE_IMAGEQUEUEIMPL_H

#include "../ImageQueue.h"
#include "QueueHead.h"
#include "SemaphoreLocker.h"


namespace imagequeue {

    /**
     * 为 ImageQueue 提供队列承载共享内存访问。
     *
     * */
    class ImageQueue::_Impl {
        std::string _name;          // Name of the image queue.
        std::string _shmname;       // Name of the shared memory segment.
        std::string _semname;       // Name of the semaphore.
        void *_addr;                // Address of the shared memory containing the queue inside the
                                    // address space of calling process.
        size_t _length;             // Length of the shared memory segment.
        sem_t *_syncSem;            // Synchronization semaphore of the queue.

        void openSemaphore();
        void openSharedMemory(QueueHead head);
        void closeSemaphore(bool unlinkName) noexcept;
        void closeSharedMemory(bool unlinkName) noexcept;

    public:
        _Impl(std::string name, uint32_t capacity, uint32_t rows, uint32_t cols, uint32_t pixelType, size_t pixelSize);

        _Impl(const _Impl &) = delete;
        _Impl(_Impl &&another) noexcept = delete;

        _Impl &operator=(const _Impl &) = delete;
        _Impl &operator=(_Impl &&another) noexcept = delete;

        ~_Impl() noexcept;

        /**
         * 获取共享内存的全局标识符。
         *
         * */
        std::string name() const noexcept;

        /**
         * 获取共享内存在调用进程中的映射基地址。
         *
         * */
        void *addr() const noexcept;

        /**
         * 申请对共享内存的互斥访问锁。
         *
         * */
        SemaphoreLocker acquireLock() const;
    };

}


#endif //IMAGEQUEUE_IMAGEQUEUEIMPL_H
