#ifndef UTILITY_IDIOM_PIMPL_H
#define UTILITY_IDIOM_PIMPL_H
#include <memory>
namespace utility {
namespace idiom {

    /**
     * === Overview ===
     * This class provides a wrapper for implementing the
     * C++ pimpl (pointer-to-implementation) idiom. This idiom
     * allows you to hide the private implementation details of
     * a class in the cpp file. This means that you don't need to
     * include libraries in your headers if they're only used in
     * the private interface.
     *
     * === Basic Usage ===
     * 1. In the header add #include "utility/idiom/pimpl.h"
     * 2. In the header replace every non-virtual function and memeber in the private
     * section of a class with the following snippet:
     *
     *     ...
     *     private:
     *         class impl;
     *         utility::idiom::pimpl<impl> m;
     *
     * 3. In the source file add the header "utility/idiom/pimpl_impl.h"
     * 4. In the source file place the following snippet at the top of the file (after includes):
     *
     *    #include "utility/idiom/pimpl_impl.h"
     *    class <ReactorName>::impl {
     *        public:
     *            <private members and functions go here>
     *    };
     *
     * === Example ===
     * This example shows how to convert a regular class to use the pimpl idiom.
     *
     * Regular class:
     * 
     *     -- .h file: --
     *     #include <SuperLibrary.h>
     *     #include <AmazingLibrary.h>
     *     class SomeReactor {
     *          public:
     *              explicit SomeReactor(NUClear::PowerPlant* powerPlant);
     *          private:
     *              void doSomethingCool();
     *
     *              SuperLibrary::Context* superLibraryContext;
     *              AmazingLibrary::Matrix myMatrix;
     *     };
     *
     *
     *     -- .cpp file: --
     *     ... regular stuff here ...
     *
     * Pimpl version:
     *
     *    -- .h file: --
     *    #include "utility/idiom/pimpl.h"
     *    class SomeReactor {
     *        public:
     *            explicit SomeReactor(NUClear::PowerPlant* powerPlant);
     *        private:
     *            class impl;
     *            utility::idiom::pimpl<impl> m;
     *    };
     *
     *    -- .cpp file: --
     *    #include "utility/idiom/pimpl_impl.h"
     *    class SomeReactor::impl {
     *        public:
     *            void doSomethingCool();
     *
     *            SuperLibrary::Context* superLibraryContext;
     *            AmazingLibrary::Matrix myMatrix;
     *    };
     *
     *    ... regular stuff here ...
     *
     *
     * The main advantage from the above sample is we no longer have SuperLibrary.h and
     * AmazingLibrary.h in the header. This means that other modules that include this header
     * no longer need to know about AmazingLibrary or SuperLibrary. This improves compile speed
     * and means that the public header doesn't change if the internal implementation does.
     *
     * === Additional Information ===
     * http://herbsutter.com/gotw/_100/
     * http://herbsutter.com/gotw/_101/
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
