/*
 * Resampling Method Configuration Wrapper
 *
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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

namespace utility::math::filter {

    /// @brief Wrapper for resampling methods
    /// @see http://users.isy.liu.se/rt/schon/Publications/HolSG2006.pdf for an explanation on each one
    struct [[nodiscard]] ResampleMethod {
        constexpr ResampleMethod() = default;

        // The possible resampling methods
        // All are disabled by default
        bool residual_enabled    = false;
        bool systematic_enabled  = false;
        bool multinomial_enabled = false;
        bool stratified_enabled  = false;

        /// @brief Checks the validity of the resample method
        /// @details To be a valid method, it must have exactly one of systematic, stratified, or multinomial enabled.
        ///          Residual resampling can be optionally enabled as well as one of those, but it is invalid on its own
        ///          That's because residual resampling requires a secondary method to resample the residual particles
        /// @retval true A valid resampling method configuration has been set
        /// @retval false An invalid resampling method configuration has been set
        [[nodiscard]] constexpr bool is_valid() const {

            if (no_methods_enabled() || disjoint_methods_enabled() || residual_method_only()) {
                return false;
            }
            return true;
        }

    private:
        /// @brief Returns true if mutually exclusive methods are enabled and false otherwise
        [[nodiscard]] constexpr bool disjoint_methods_enabled() const {
            // clang-format off
            return     (systematic_enabled && multinomial_enabled)
                    || (systematic_enabled && stratified_enabled)
                    || (multinomial_enabled && stratified_enabled);
            // clang-format on
        }

        /// @brief Returns true if residual sampling is the only method enabled and false otherwise
        [[nodiscard]] constexpr bool residual_method_only() const {
            return residual_enabled && !(systematic_enabled || multinomial_enabled || stratified_enabled);
        }

        /// @brief Returns true if there aren't any methods enabled and false otherwise
        [[nodiscard]] constexpr bool no_methods_enabled() const {
            return !(residual_enabled || systematic_enabled || multinomial_enabled || stratified_enabled);
        }
    };

}  // namespace utility::math::filter
