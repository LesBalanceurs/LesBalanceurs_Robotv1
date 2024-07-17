/****************************************************************************************
Nom du fichier : ChangeSummator.h
Auteur : Mathieu Durand      
Date de création : 17 juillet 2024

****************************************************************************************/

#ifndef CHANGE_SUMMATOR_H
#define CHANGE_SUMMATOR_H

#define DEFAULT_SUM_VALUE 0

/**
 * @class ChangeSummator
 * @brief Class for summing change, similar to the concept of displacement
 */

template <typename N> 
class ChangeSummator
{
    public:
        ChangeSummator(N initialValue);
        ChangeSummator();

        N getSum();
        void setSum(N sum);

        void track(N newValue);
        void updateWithChange(N change);
    
    private:
        N sum;

    protected:
        N value;
};

template class ChangeSummator<double>;
template class ChangeSummator<float>;
template class ChangeSummator<int>;
template class ChangeSummator<unsigned int>;
template class ChangeSummator<long>;
template class ChangeSummator<unsigned long>;
template class ChangeSummator<short>;
template class ChangeSummator<unsigned short>;
template class ChangeSummator<long long>;
template class ChangeSummator<unsigned long long>;


#endif // CHANGE_SUMMATOR_H
