#ifndef COMMON_H
#define COMMON_H

#include <QtCore>

/* round number n to d decimal points */
inline double round(double n, unsigned d){return floor(n * pow(10., d) + .5) / pow(10., d);}

inline bool isvalid(QPointF point) { return !(isnan(point.x()) || isnan(point.y()) || isinf(point.x()) || isinf(point.y())); }

#endif // COMMON_H
