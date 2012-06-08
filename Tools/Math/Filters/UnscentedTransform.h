/*! @file UnscentedTransform.h
 @brief Declaration of UnscentedTransform class
 
 @class UnscentedTransform
 @brief An implementation of the Unscented transform calculations.
 
 @author Steven Nicklin
 
 Copyright (c) 2012 Steven Nicklin
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

class UnscentedTransform
{
public:
    /*!
     * @brief Constructor
     *
     * This constructor sets the Unscented transform parameters.
     *
     * @param L the L variable. This is the number of states in the filter.
     * @param alpha the alpha variable. This determines the spread of the sigma points about the mean. This is usually a small positive value eg 0.001.
     * @param kappa the kappa variable. This is the secondary scaling parameter, usually set to 0 or 3-L.
     * @param beta the beta variable. This is used to incorporate prior knowledge of the distribution. For gaussian distributions 2 is optimal.
     */
    UnscentedTransform(unsigned int L, double alpha=0.001f, double kappa=0.f, double beta=2.f): m_L(L), m_alpha(alpha), m_kappa(kappa), m_beta(beta)
    {}

    
    /*!
     * @brief Function used to set the Unscented transform parameters.
     *
     * The text should describe the link as follows:
     * Joint Number, Joint Name, alpha, a, thetaOffset, d
     *
     * @param L the L variable. This is the number of states in the filter.
     * @param alpha the alpha variable. This determines the spread of the sigma points about the mean. This is usually a small positive value eg 0.001.
     * @param kappa the kappa variable. This is the secondary scaling parameter, usually set to 0 or 3-L.
     * @param beta the beta variable. This is used to incorporate prior knowledge of the distribution. For gaussian distributions 2 is optimal.
     */
    void setValue(unsigned int L, double alpha, double kappa, double beta)
    {
        m_L = L;
        m_alpha = alpha;
        m_kappa = kappa;
        m_beta = beta;
    }

    /*!
     * @brief Get function used to retrieve the L variable.
     * @return The current value of L.
     */
    unsigned int L() const {return m_L;}

    /*!
     * @brief Get function used to retrieve the alpha variable.
     * @return The current value of alpha.
     */
    double alpha() const {return m_alpha;}
    
    /*!
     * @brief Get function used to retrieve the kappa variable.
     * @return The current value of kappa.
     */
    double kappa() const {return m_kappa;}

    /*!
     * @brief Get function used to retrieve the beta variable.
     * @return The current value of beta.
     */
    double beta() const {return m_beta;}

    /*!
     * @brief Calculate the current lambda value, based on the other parameters.
     * @return The lambda value.
     */    
    
    double lambda() const
    {
        return pow(m_alpha,2) * (m_L + m_kappa) - m_L;
    }

    unsigned int totalSigmaPoints() const
    {
        return 2 * L() + 1;
    }

    /*!
     * @brief Calculate the ith mean weighting value, where i is the index.
     * @param The index of the weight to be caluclated.
     * @return The ith mean weight.
     */
    double Wm(unsigned int index) const
    {
        double weight = 0.0f;
        switch(index)
        {
            case 0:
                weight = lambda() / (L() + lambda());
                break;
            default:
                weight = 1.0f / (2.0f * (L() + lambda()));
                break;
        }
        return weight;
    }

    /*!
     * @brief Calculate the ith mean covariance value, where i is the index.
     * @param The index of the covariance to be caluclated.
     * @return The ith covariance weight.
     */
    double Wc(unsigned int index) const
    {
        double weight = 0.0f;
        switch(index)
        {
            case 0:
                weight = lambda() / (L() + lambda()) + (1.0f - alpha()*alpha() + beta());
                break;
            default:
                weight = 1.0f / (2.0f * (L() + lambda()));
                break;
        }
        return weight;
    }

    double covarianceSigmaWeight() const
    {
        return L() + lambda();
    }

private:
    unsigned int m_L;
    double m_alpha;
    double m_kappa;
    double m_beta;
};
