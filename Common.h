#ifndef COMMON_H
#define COMMON_H

/* round number n to d decimal points */
inline double round(double n, unsigned d){return floor(n * pow(10., d) + .5) / pow(10., d);}

#endif // COMMON_H
