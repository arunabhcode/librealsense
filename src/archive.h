// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once
#ifndef LIBREALSENSE_ARCHIVE_H
#define LIBREALSENSE_ARCHIVE_H

#include "types.h"
#include <atomic>

namespace rsimpl
{
    // Defines general frames storage model
    class frame_archive
    {
    public:

        // Define a movable but explicitly noncopyable buffer type to hold our frame data
        struct frame
        {
        private:
            // TODO: check boost::intrusive_ptr or an alternative
            std::atomic<int> ref_count; // the reference count is on how many times this placeholder has been observed (not lifetime, not content)
            frame_archive * owner; // pointer to the owner to be returned to by last observe
            frame_continuation on_release;

        public:
            std::vector<byte> data;
            int timestamp = 0;
            int frame_number = 0;
			long long system_time = 0;

            explicit frame() : ref_count(0), owner(nullptr), timestamp(), frame_number(), on_release() {}
            frame(const frame & r) = delete;
            frame(frame && r) 
                : ref_count(r.ref_count.exchange(0)), 
                  owner(r.owner), on_release() 
            {
                *this = std::move(r); // TODO: This is not very safe, refactor later
            }

            frame & operator=(const frame & r) = delete;
            frame& operator=(frame&& r)
            {
                data = move(r.data);
                timestamp = r.timestamp;
                owner = r.owner;
                ref_count = r.ref_count.exchange(0);
                on_release = std::move(r.on_release);
                frame_number = r.frame_number;
				system_time = r.system_time;
                return *this;
            }

            const byte* get_frame_data() const;

            void acquire() { ref_count.fetch_add(1); }
            void release();
            frame* publish();
            void update_owner(frame_archive * new_owner) { owner = new_owner; }
            void attach_continuation(frame_continuation&& continuation) { on_release = std::move(continuation); }
        };

        class frame_ref // esentially an intrusive shared_ptr<frame>
        {
            frame * frame_ptr;
        public:
            frame_ref() : frame_ptr(nullptr) {}

            explicit frame_ref(frame* frame) : frame_ptr(frame)
            {
                if (frame) frame->acquire();
            }

            frame_ref(const frame_ref& other) : frame_ptr(other.frame_ptr)
            {
                if (frame_ptr) frame_ptr->acquire();
            }

            frame_ref(frame_ref&& other) : frame_ptr(other.frame_ptr)
            {
                other.frame_ptr = nullptr;
            }

            frame_ref& operator =(frame_ref other)
            {
                swap(other);
                return *this;
            }

            ~frame_ref()
            {
                if (frame_ptr) frame_ptr->release();
            }

            void swap(frame_ref& other)
            {
                std::swap(frame_ptr, other.frame_ptr);
            }

            const byte* get_frame_data() const;
            int get_frame_timestamp() const;
            int get_frame_number() const;
	        long long get_frame_system_time() const;
        };

        class frameset
        {
            frame_ref buffer[RS_STREAM_NATIVE_COUNT];
        public:

            frame_ref detach_ref(rs_stream stream);
            void place_frame(rs_stream stream, frame&& new_frame);

            const byte * get_frame_data(rs_stream stream) const { return buffer[stream].get_frame_data(); }
            int get_frame_timestamp(rs_stream stream) const { return buffer[stream].get_frame_timestamp(); }
            int get_frame_number(rs_stream stream) const { return buffer[stream].get_frame_number(); }
			long long get_frame_system_time(rs_stream stream) const { return buffer[stream].get_frame_system_time(); }

            void cleanup();

        };

    private:
        // This data will be left constant after creation, and accessed from all threads
        subdevice_mode_selection modes[RS_STREAM_NATIVE_COUNT];
        
        small_heap<frame, RS_USER_QUEUE_SIZE> published_frames;
        small_heap<frameset, RS_USER_QUEUE_SIZE> published_sets;
        small_heap<frame_ref, RS_USER_QUEUE_SIZE> detached_refs;

    protected:
        frame backbuffer[RS_STREAM_NATIVE_COUNT]; // recieve frame here
        std::vector<frame> freelist; // return frames here
        std::recursive_mutex mutex;

    public:
        frame_archive(const std::vector<subdevice_mode_selection> & selection);

        // Safe to call from any thread
        bool is_stream_enabled(rs_stream stream) const { return modes[stream].mode.pf.fourcc != 0; }
        const subdevice_mode_selection & get_mode(rs_stream stream) const { return modes[stream]; }

        void release_frameset(frameset * frameset)
        {
            published_sets.deallocate(frameset);
        }
        frameset * clone_frameset(frameset * frameset);

        void unpublish_frame(frame * frame);
        frame * publish_frame(frame && frame);

        frame_ref * detach_frame_ref(frameset * frameset, rs_stream stream);
        frame_ref * clone_frame(frame_ref * frameset);
        void release_frame_ref(frame_ref * ref)
        {
            detached_refs.deallocate(ref);
        }

        // Frame callback thread API
        byte * alloc_frame(rs_stream stream, int timestamp, int frameCounter, long long system_time, bool requires_memory);
        frame_ref * track_frame(rs_stream stream);
        void attach_continuation(rs_stream stream, frame_continuation&& continuation);

        virtual void flush();

        virtual ~frame_archive() {};
    };
}

#endif