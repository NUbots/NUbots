#ifndef UTILITY_TYPE_TRAITS_HAS_ID_HPP
#define UTILITY_TYPE_TRAITS_HAS_ID_HPP

#include <cstdint>
#include <type_traits>

namespace utility::type_traits {

    /**
     * @brief SFINAE struct to test if the passed class has an ID field used for subtyping
     *
     * @tparam T the class to check
     */
    template <typename T>
    struct has_id {
    private:
        using yes = std::true_type;
        using no  = std::false_type;

        template <typename U>
        static auto test(int) -> decltype((std::declval<U>().id, yes()));
        template <typename>
        static no test(...);

    public:
        static constexpr bool value = std::is_same<decltype(test<T>(0)), yes>::value;
    };

}  // namespace utility::type_traits

#endif  // UTILITY_TYPE_TRAITS_HAS_ID_HPP
