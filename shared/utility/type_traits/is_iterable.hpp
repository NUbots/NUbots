#ifndef UTILITY_TYPE_TRAITS_IS_ITERABLE
#define UTILITY_TYPE_TRAITS_IS_ITERABLE

#include <utility>

namespace utility::type_traits {

    /**
     * @brief SFINAE struct to test if the passed class has a begin and an end.
     *
     * @tparam T the class to check
     *
     * @note Replaced by range concepts in c++20
     */
    template <typename T>
    struct is_iterable {
    private:
        using yes = std::true_type;
        using no  = std::false_type;

        template <typename U>
        static auto test_begin(int) -> decltype(std::declval<U>().begin(), yes());
        template <typename>
        static no test_begin(...);

        template <typename U>
        static auto test_end(int) -> decltype(std::declval<U>().end(), yes());
        template <typename>
        static no test_end(...);

    public:
        static constexpr bool value =
            std::is_same<decltype(test_begin<T>(0)), yes>::value && std::is_same<decltype(test_end<T>(0)), yes>::value;
    };
}  // namespace utility::type_traits

#endif  // UTILITY_TYPE_TRAITS_IS_ITERABLE
