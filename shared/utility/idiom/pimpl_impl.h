#ifndef UTILITY_IDIOM_PIMPL_IMPL_H
#define UTILITY_IDIOM_PIMPL_IMPL_H
#include <utility>

namespace utility {
namespace idiom {
    template <typename T>
    pimpl<T>::pimpl() : m{ new T{} } {}

    template <typename T>
    template <typename... TArgs>
    pimpl<T>::pimpl( TArgs&&... args ) :
        m{ new T{ std::forward<TArgs>(args)... } } {}

    template <typename T>
    pimpl<T>::~pimpl() {}

    template <typename T>
    T* pimpl<T>::operator->() { return m.get(); }

    template <typename T>
    T& pimpl<T>::operator*() { return *m.get(); }
}
}
#endif
