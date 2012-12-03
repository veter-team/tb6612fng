#ifndef __DIVCONST_H
#define __DIVCONST_H

// Efficiently divides by 100 using multiplication and shift only
int ldiv100(long long dividend);

// Signed divide by 100
int divs100(int n);

// Unsigned divide by 1000
unsigned divu1000(unsigned n);

// Signed divide by 1000
int divs1000(int n);

#endif //__DIVCONST_H
