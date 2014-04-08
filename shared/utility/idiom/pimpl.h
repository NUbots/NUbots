/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

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
