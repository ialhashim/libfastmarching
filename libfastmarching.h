#ifndef LIBFASTMARCHING_H
#define LIBFASTMARCHING_H

#include <array>
#include <QImage>
#include <QPainter>

struct libfastmarching
{
	typedef std::vector< std::array<double, 2> > Path;
	typedef std::vector<std::vector<double> > DistanceGrid;

    static QImage fmm(QImage input, QPoint from, QPoint to, Path & path);
	static QImage fsm(QImage input, DistanceGrid & distances);
};

#endif // LIBFASTMARCHING_H
