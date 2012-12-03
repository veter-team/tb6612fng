/*
I did not write these functions myself. Instead, thanks goes to Dmitri
Shelenin for ldiv100() and the rest of the functions are copy-pasted
from pdf file authored by Hank:
http://www.hackersdelight.org/divcMore.pdf. Thanks Hank!
*/

#include "divconst.h"


int 
ldiv100(long long dividend)
{
  long long divisor = 0x28f5c29;
  return ((divisor * dividend)>>32) & 0xffffffff;
}


int 
divs100(int n)
{
  int q, r;
  n = n + (n>>31 & 99);
  q = (n >> 1) + (n >> 3) + (n >> 6) - (n >> 10) +
    (n >> 12) + (n >> 13) - (n >> 16);
  q = q + (q >> 20);
  q = q >> 6;
  r = n - q*100;
  return q + ((r + 28) >> 7);
  // return q + (r > 99);
}


unsigned 
divu1000(unsigned n)
{
  unsigned q, r, t;
  t = (n >> 7) + (n >> 8) + (n >> 12);
  q = (n >> 1) + t + (n >> 15) + (t >> 11) + (t >> 14);
  q = q >> 9;
  r = n - q*1000;
  return q + ((r + 24) >> 10);
//  return q + (r > 999);
}


int 
divs1000(int n)
{
  int q, r, t;
  n = n + (n>>31 & 999);
  t = (n >> 7) + (n >> 8) + (n >> 12);
  q = (n >> 1) + t + (n >> 15) + (t >> 11) + (t >> 14) 
      + (n >> 26) + (t >> 21);
  q = q >> 9;
  r = n - q*1000;
  return q + ((r + 24) >> 10);
//  return q + (r > 999);
}

