#ifndef LIBFASTMARCHING_H
#define LIBFASTMARCHING_H

#include <array>
#include <QImage>
#include <QPainter>

struct libfastmarching
{
	typedef std::vector< std::array<double, 2> > Path;
    typedef std::vector< std::vector<double> > DistanceGrid;

    static QImage fmm(QImage input, QPoint from, QPoint to, Path & path, bool isVisualize = true);
    static QImage fm2star(QImage input, QPoint from, QPoint to, Path & path, bool isVisualize = true);
	static QImage fsm(QImage input, DistanceGrid & distances);

    struct DynamicGrid{
        DynamicGrid(QImage input);
        QImage walls;
        void modifyWalls(libfastmarching::Path path, int thickness);
        Path bestPath(QPoint from, QPoint to, QImage & visualization);
    };
};

#endif // LIBFASTMARCHING_H
