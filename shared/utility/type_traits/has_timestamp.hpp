#ifndef UTILITY_TYPE_TRAITS_HAS_TIMESTAMP_HPP
#define UTILITY_TYPE_TRAITS_HAS_TIMESTAMP_HPP

#include <cstdint>
#include <type_traits>

namespace utility::type_traits {

    /**
     * @brief SFINAE struct to test if the passed class has a timestamp field
     *
     * @tparam T the class to check
     */
    template <typename T>
    struct has_timestamp {
    private:
        using yes = std::true_type;
        using no  = std::false_type;

        template <typename U>
        static auto test(int) -> decltype((std::declval<U>().timestamp, yes()));
        template <typename>
        static no test(...);

    public:
        static constexpr bool value = std::is_same<decltype(test<T>(0)), yes>::value;
    };

}  // namespace utility::type_traits

#endif  // UTILITY_TYPE_TRAITS_HAS_TIMESTAMP_HPP
