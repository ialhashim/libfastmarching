#include "libfastmarching.h"

#include "fmm/fmm.hpp"
#include "fm2/fm2star.hpp"
#include "fmm/fsm.hpp"
#include "gradientdescent/gradientdescent.hpp"
#include "io/maploader.hpp"
const  double utils::COMP_MARGIN = DBL_EPSILON * 1e5;

using namespace std;

typedef FMCell CellType;
typedef nDGridMap<FMCell, 2> GridType;
Q_DECLARE_METATYPE( FMCell )
Q_DECLARE_METATYPE( GridType )

static GridType makeGrid(QImage input)
{
	nDGridMap<FMCell, 2> grid;

	// Fill grid from 'input' image
	{
		std::vector<unsigned int> obs;

		cimg_library::CImg<double> img(input.width(), input.height(), 1, 1, 0);

		std::array<unsigned int, 2> dimsize;
		dimsize[0] = img.width();
		dimsize[1] = img.height();
		grid.resize(dimsize);

		cimg_forXY(img, x, y) {
			img(x, y) = qRed(input.pixel(x, y));
		}

		// Filling the grid flipping Y dim. We want bottom left to be the (0,0).
		cimg_forXY(img, x, y) {
			double occupancy = img(x, y) / 255;
			unsigned int idx = img.width()*(img.height() - y - 1) + x;
			grid[idx].setOccupancy(occupancy);
			if (grid[idx].isOccupied())
				obs.push_back(idx);
		}
		grid.setOccupiedCells(std::move(obs));
	}

	return grid;
}

static QImage drawGrid(GridType& grid, bool isValue = true)
{
    std::array<unsigned int,2> d = grid.getDimSizes();
    double max_val = grid.getMaxValue();
    CImg<double> img(d[0],d[1],3,1,0);

    if(isValue)
    {
        cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getValue()/max_val*255; }
        img.map( CImg<double>::cool_LUT256() );
    }
    else
    {
        cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getOccupancy()*255; }
    }

    QImage imgQt(img.width(), img.height(), QImage::Format_ARGB32);

    cimg_forXY( img, x, y )
    {
       auto R = img( x, y, 0 );
       auto G = img( x, y, 1 );
       auto B = img( x, y, 2 );
       imgQt.setPixel( x, y, qRgb(R,G,B) );
    }

    return imgQt;
}

QImage libfastmarching::fmm(QImage input, QPoint from, QPoint to, Path & path, bool isVisualize)
{
    QImage output;

	auto grid = makeGrid(input);

    std::array<unsigned int, 2> init_points, goal;
    unsigned int goal_idx;

    init_points[0] = from.x();
	init_points[1] = (input.height() - 1) - from.y();

    goal[0] = to.x();
	goal[1] = (input.height() - 1) - to.y();

    // Compute using the Fast Marching Method (FMM)
    FMM< nDGridMap<FMCell, 2> > fmm;
    fmm.setEnvironment(&grid);
    fmm.setInitialAndGoalPoints(init_points, goal);
    fmm.compute();

    if(isVisualize) output = drawGrid( grid );

    // Compute path
    std::vector <double> path_velocity;
    GradientDescent< nDGridMap<FMCell, 2> > grad;
    grid.coord2idx(goal, goal_idx);
    grad.apply(grid, goal_idx, path, path_velocity);
    Q_UNUSED(grad);

    // Fix path's 'y'
    for (auto & p : path) p[1] = (input.height() - 1) - p[1];

    if( isVisualize )
    {
        // Draw path
        if (path.size())
        {
            QPainter painter(&output);
            auto prev_p = path.front();

            painter.setPen(QPen(Qt::black, 3));
            for (auto p : path){
                painter.drawLine(QPoint(prev_p[0], prev_p[1]), QPoint(p[0], p[1]));
                prev_p = p;
            }
        }
    }
    return output;
}

QImage libfastmarching::fm2star(QImage input, QPoint from, QPoint to, libfastmarching::Path &path, bool isVisualize)
{
    QImage output;

    auto grid = makeGrid(input);

    std::vector<unsigned int> init_points_idx;
    std::array<unsigned int,2> init_points, goal;
    unsigned int init_idx, goal_idx;

    init_points[0] = from.x();
    init_points[1] = (input.height() - 1) - from.y();
    grid.coord2idx(init_points, init_idx);
    init_points_idx.push_back(init_idx);

    goal[0] = to.x();
    goal[1] = (input.height() - 1) - to.y();
    grid.coord2idx(goal, goal_idx);

    FM2Star< nDGridMap<FMCell, 2> > fm2star;
    fm2star.setEnvironment(&grid);
    fm2star.setInitialAndGoalPoints(init_points_idx, goal_idx);
    fm2star.compute();

    if(isVisualize) output = drawGrid( grid, false );

    std::vector <double> path_velocity;
    fm2star.computePath(&path, &path_velocity);

    // Fix path's 'y'
    for (auto & p : path) p[1] = (input.height() - 1) - p[1];

    if( isVisualize )
    {
        // Draw path
        if (path.size())
        {
            QPainter painter(&output);
            auto prev_p = path.front();

            painter.setPen(QPen(Qt::black, 3));
            for (auto p : path){
                painter.drawLine(QPoint(prev_p[0], prev_p[1]), QPoint(p[0], p[1]));
                prev_p = p;
            }
        }
    }
    return output;
}

QImage libfastmarching::fsm(QImage input, DistanceGrid & distances)
{
	auto grid = makeGrid(input);

	FSM< nDGridMap<FMCell, 2> > fsm;
	fsm.setEnvironment(&grid);
	std::vector<unsigned int> obs;
	grid.getOccupiedCells(obs);
	fsm.setInitialPoints(obs);
	fsm.compute();

	distances.resize(input.height(), std::vector<double>(input.width(), 0));

	QImage output;
	{
		std::array<unsigned int, 2> d = grid.getDimSizes();
		double max_val = grid.getMaxValue();
		CImg<double> img(d[0], d[1], 1, 1, 0);
		// Filling the image flipping Y dim. We want now top left to be the (0,0).
		cimg_forXY(img, x, y) { 
			img(x, y) = grid[img.width()*(img.height() - y - 1) + x].getValue() / max_val * 255; 
			distances[y][x] = grid[img.width()*(img.height() - y - 1) + x].getValue();
		}

		img.map(CImg<double>::jet_LUT256());
		//img.display("", false);

		QImage imgQt(img.width(), img.height(), QImage::Format_ARGB32);

		cimg_forXY(img, x, y)
		{
			auto R = img(x, y, 0);
			auto G = img(x, y, 1);
			auto B = img(x, y, 2);
			imgQt.setPixel(x, y, qRgb(R, G, B));
		}

		output = imgQt;
	}

	return output;
}

libfastmarching::DynamicGrid::DynamicGrid(QImage input)
{
    walls = input;
}

void libfastmarching::DynamicGrid::modifyWalls(libfastmarching::Path path, int thickness)
{
    QPolygon qpoly;
    for(auto p : path) qpoly << QPoint(p[0], p[1]);

    QPainter p(&walls);
    p.setRenderHint(QPainter::Antialiasing, false);
    p.setPen(QPen(Qt::black, thickness, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPolyline(qpoly);
}

libfastmarching::Path libfastmarching::DynamicGrid::bestPath(QPoint from, QPoint to, QImage &visualization)
{
    libfastmarching::Path resultingPath;
    visualization = fm2star(walls, from, to, resultingPath, true);
    return resultingPath;
}
