//
// Created by lancern on 19-3-13.
//

#include "ImageQueueImpl.h"
#include "util.h"
#include "traceutil.h"

#include <cstring>

#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>

#define SEM_NAME_SUFFIX     "_semlock"
#define SHM_NAME_SUFFIX     "_shm"


namespace imagequeue {

    std::string prepareSemaphoreName(const std::string &name) {
        auto result = std::string();
        if (name.empty() || name.front() != '/')
            result.push_back('/');

        result.append(name);
        result.append(SEM_NAME_SUFFIX);

        return result;
    }

    std::string prepareSharedMemoryName(const std::string &name) {
        auto result = std::string();
        if (name.empty() || name.front() != '/')
            result.push_back('/');

        result.append(name);
        result.append(SHM_NAME_SUFFIX);

        return result;
    }

    void ImageQueue::_Impl::openSemaphore() {
        TRACE_ARGS("Opening semaphore with name \"%s\"", _semname.data());

        _syncSem = ::sem_open(_semname.data(), O_CREAT, S_IRWXU | S_IRWXG | S_IRWXO, 1);
        if (_syncSem == SEM_FAILED)
            util::throwSysErr();
    }

    void ensureQueueHeadConsistency(const QueueHead &qh1, const QueueHead &qh2) {
        if (!formatEqual(qh1, qh2))
            throw std::invalid_argument("Formats of queues not consistent.");
    }

    void ImageQueue::_Impl::openSharedMemory(QueueHead head) {
        SEMAPHORE_LOCK_BEGIN_VAR(semlock, _syncSem)

            TRACE_ARGS("Opening shared memory with name \"%s\"", _shmname.data());
            auto shmFd = ::shm_open(_shmname.data(),
                                    O_CREAT | O_RDWR,
                                    S_IRWXU | S_IRWXG | S_IRWXO);
            if (shmFd == -1)
                util::throwSysErr();

            // Test if the shared memory was previously opened and initialized by another process.
            auto initialized = util::fsize(shmFd) != 0;

            auto fdCloser = [&shmFd] () -> void { ::close(shmFd); };

            // Calculate the minimal size of the shared memory containing the queue.
            _length = sizeof(QueueHead) + head.rows * head.cols * head.pixelSize * head.capacity;
            // And set the size of the shared memory segment.
            if (::ftruncate(shmFd, _length) == -1)
                util::cleanupAndThrowSysErr(fdCloser);

            _addr = ::mmap(nullptr, _length, PROT_READ | PROT_WRITE, MAP_SHARED, shmFd, 0);
            if (_addr == MAP_FAILED)
                util::cleanupAndThrowSysErr(fdCloser);

            fdCloser();

            if (initialized) {
                // Compare the given head against the existing head.
                auto existingHead = *reinterpret_cast<QueueHead *>(_addr);
                try {
                    ensureQueueHeadConsistency(head, existingHead);
                } catch (std::invalid_argument &) {
                    semlock.release();
                    closeSemaphore(false);
                    closeSharedMemory(false);
                    throw;
                }
            } else {
                TRACE_MSG("Shared memory segment is not initialized. Initializing it...");
                memcpy(_addr, &head, sizeof(QueueHead));
            }

        SEMAPHORE_LOCK_END
    }

    void ImageQueue::_Impl::closeSemaphore(bool unlinkName) noexcept {
        if (_syncSem) {
            if (unlinkName)
                ::sem_unlink(_semname.data());
            ::sem_close(_syncSem);
            _syncSem = nullptr;
        }
    }

    void ImageQueue::_Impl::closeSharedMemory(bool unlinkName) noexcept {
        if (unlinkName)
            ::shm_unlink(_shmname.data());

        ::munmap(_addr, _length);
        _addr = nullptr;
    }

    ImageQueue::_Impl::_Impl(std::string name, uint32_t capacity, uint32_t rows, uint32_t cols, uint32_t pixelType, size_t pixelSize)
        : _name(std::move(name)),
          _shmname(prepareSharedMemoryName(_name)),
          _semname(prepareSemaphoreName(_name)),
          _addr(nullptr),
          _length(0),
          _syncSem(nullptr) {
        openSemaphore();
        openSharedMemory(makeQueueHead(capacity, rows, cols, pixelType, pixelSize));
    }

    ImageQueue::_Impl::~_Impl() noexcept {
        if (!_addr || _addr == MAP_FAILED)
            return;

        closeSharedMemory(true);
        closeSemaphore(true);
    }

    std::string ImageQueue::_Impl::name() const noexcept {
        return _name;
    }

    void *ImageQueue::_Impl::addr() const noexcept {
        return _addr;
    }

    SemaphoreLocker ImageQueue::_Impl::acquireLock() const {
        return SemaphoreLocker(_syncSem);
    }

}
