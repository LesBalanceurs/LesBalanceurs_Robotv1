/****************************************************************************************
Nom du fichier : ChangeSummator.cpp
Auteur : Mathieu Durand        
Date de création : 17 juillet 2024
  
****************************************************************************************/
#include "ChangeSummator.h"

/*============================================================================
								PUBLIC
============================================================================*/

template <typename N> 
ChangeSummator<N>::ChangeSummator(N initialValue) {
    this->setSum(initialValue);
}

template <typename N> 
ChangeSummator<N>::ChangeSummator(): ChangeSummator(DEFAULT_SUM_VALUE) {}

template <typename N> 
N ChangeSummator<N>::getSum() {
    return this->sum;
}

template <typename N> 
void ChangeSummator<N>::setSum(N sum) {
    this->sum = sum;
}


template <typename N> 
void ChangeSummator<N>::track(N newValue) {
    setSum(getSum() + (newValue - this->value));
    this->value = newValue;
}

template <typename N> 
void ChangeSummator<N>::updateWithChange(N change) {
    setSum(getSum() + change);
    this->value = this->value + change;
}