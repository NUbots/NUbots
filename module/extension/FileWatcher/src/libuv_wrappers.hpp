#ifndef MODULE_EXTENSION_FILEWATCHER_LIBUV_WRAPPERS_HPP
#define MODULE_EXTENSION_FILEWATCHER_LIBUV_WRAPPERS_HPP

#include <memory>
#include <uv.h>

namespace module::extension::uv {

    struct loop_t {
    public:
        loop_t() {
            uv_loop_init(&loop);
            closed = false;
        }
        void close() {
            if (!closed) {
                closed = true;
                uv_loop_close(&loop);
            }
        }
        ~loop_t() {
            close();
        }
        int run(uv_run_mode mode) {
            return uv_run(&loop, mode);
        }
        uv_loop_t* get() {
            return &loop;
        }

    private:
        uv_loop_t loop;
        bool closed;
    };

    struct async_t {
    public:
        async_t(loop_t loop, uv_async_cb async_cb, void* data = nullptr) {
            uv_async_init(loop.get(), &async, async_cb);
            async.data = data;
            closed     = false;
        }
        void close() {
            if (!closed) {
                closed = true;
                uv_close(reinterpret_cast<uv_handle_t*>(&async), [](uv_handle_t* /* handle */) {});
            }
        }
        ~async_t() {
            close();
        }
        void send() {
            uv_async_send(&async);
        }

    private:
        uv_async_t async;
        bool closed;
    };

    struct fs_event_t {
    public:
        fs_event_t(loop_t loop, void* data) {
            uv_fs_event_init(loop.get(), &fs_event);
            fs_event.data = data;
            closed        = false;
        }
        void close() {
            if (!closed) {
                closed = true;
                uv_close(reinterpret_cast<uv_handle_t*>(&fs_event), [](uv_handle_t* /* handle */) {});
            }
        }
        ~fs_event_t() {
            close();
        }
        /// We ignore the flag variable as it is not implemented
        void start(uv_fs_event_cb cb, std::string& path) {
            uv_fs_event_start(&fs_event, cb, path.c_str(), 0);
        }

    private:
        uv_fs_event_t fs_event;
        bool closed;
    };
}  // namespace module::extension::uv

#endif  // MODULE_EXTENSION_FILEWATCHER_LIBUV_WRAPPERS_HPP
