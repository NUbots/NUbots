#ifndef UTILITY_IDIOM_PIMPL_H
#define UTILITY_IDIOM_PIMPL_H
#include <memory>
namespace utility {
namespace idiom {

    /**
     * This class provides a wrapper for implementing the
     * C++ pimpl (pointer-to-implementation) idiom. This idiom
     * allows you to hide the private implementation details of
     * a class in the cpp file. This means that you don't need to
     * include libraries in your headers if they're only used in
     * the private interface.
     *
     * see: http://herbsutter.com/gotw/_100/
     * see: http://herbsutter.com/gotw/_101/
     */
    template <typename T>
    class pimpl {
        public:
            pimpl();

            template <typename... TArgs>
            pimpl( TArgs&&... );

            ~pimpl();

            T* operator->();
            T& operator*();
        private:
            std::unique_ptr<T> m;
    };
}
}
#endif
