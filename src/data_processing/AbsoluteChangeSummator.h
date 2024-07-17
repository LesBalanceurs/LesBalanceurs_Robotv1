/****************************************************************************************
Nom du fichier : AbsoluteChangeSummator.h
Auteur : Mathieu Durand      
Date de création : 11 juillet 2024

****************************************************************************************/

#ifndef ABSOLUTE_CHANGE_SUMMATOR_H
#define ABSOLUTE_CHANGE_SUMMATOR_H

#include "ChangeSummator.h"

/**
 * @class AbsoluteChangeSummator
 * @brief  Class for summing absolute change, similar to the concept of distance
 */

template <typename N> 
class AbsoluteChangeSummator: public ChangeSummator<N>
{
    public:
        AbsoluteChangeSummator(N initialValue);
        AbsoluteChangeSummator();

        void track(N newValue);
        void updateWithChange(N change);
};

template class AbsoluteChangeSummator<double>;
template class AbsoluteChangeSummator<float>;
template class AbsoluteChangeSummator<int>;
template class AbsoluteChangeSummator<unsigned int>;
template class AbsoluteChangeSummator<long>;
template class AbsoluteChangeSummator<unsigned long>;
template class AbsoluteChangeSummator<short>;
template class AbsoluteChangeSummator<unsigned short>;
template class AbsoluteChangeSummator<long long>;
template class AbsoluteChangeSummator<unsigned long long>;


#endif // ABSOLUTE_CHANGE_SUMMATOR_H
