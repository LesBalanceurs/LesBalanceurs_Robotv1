/****************************************************************************************
Nom du fichier : AbsoluteChangeSummator.cpp
Auteur : Mathieu Durand        
Date de création : 17 juillet 2024
  
****************************************************************************************/
#include "AbsoluteChangeSummator.h"
#include "Arduino.h"

/*============================================================================
								PUBLIC
============================================================================*/

template <typename N> 
AbsoluteChangeSummator<N>::AbsoluteChangeSummator(N initialValue): ChangeSummator<N>(initialValue) {}

template <typename N> 
AbsoluteChangeSummator<N>::AbsoluteChangeSummator(): ChangeSummator<N>() {}

template <typename N> 
void AbsoluteChangeSummator<N>::track(N newValue) {
    setSum(getSum() + abs(newValue - this->value));
    this->value = newValue;
}

template <typename N> 
void AbsoluteChangeSummator<N>::updateWithChange(N change) {
    setSum(getSum() + abs(change));
    this->value = this->value + change;
}