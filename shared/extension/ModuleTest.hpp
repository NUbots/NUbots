/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef EXTENSION_MODULETEST_HPP
#define EXTENSION_MODULETEST_HPP

#include <nuclear>

namespace extension {

    template <typename Module>
    class ModuleTest : NUClear::PowerPlant {
    public:
        ModuleTest() = delete;

        explicit ModuleTest(Module m) : module(m), NUClear::PowerPlant(getSingleThreadConfig()) {}
        ~ModuleTest() {
            shutdown();
        }
        ModuleTest(ModuleTest& other)  = delete;
        ModuleTest(ModuleTest&& other) = delete;
        ModuleTest& operator=(ModuleTest& other) = delete;
        ModuleTest&& operator=(ModuleTest&& other) = delete;

    private:
        static const NUClear::PowerPlant::Configuration& getSingleThreadConfig() {
            NUClear::PowerPlant::Configuration cfg;
            cfg.thread_count = 1;
            return cfg;
        }

        Module module;
    };

}  // namespace extension

#endif  // EXTENSION_MODULETEST_HPP
