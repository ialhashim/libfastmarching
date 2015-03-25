#include "libfastmarching.h"

#include "fmm/fmm.hpp"
#include "fmm/fsm.hpp"
#include "gradientdescent/gradientdescent.hpp"
#include "io/maploader.hpp"
const  double utils::COMP_MARGIN = DBL_EPSILON * 1e5;

using namespace std;

static nDGridMap<FMCell, 2> makeGrid(QImage input)
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

QImage libfastmarching::fmm(QImage input, QPoint from, QPoint to, Path & path)
{
	auto grid = makeGrid(input);

	std::array<unsigned int, 2> init_points, goal;
	unsigned int goal_idx;

    init_points[0] = from.x();
	init_points[1] = (input.height() - 1) - from.y();

    goal[0] = to.x();
	goal[1] = (input.height() - 1) - to.y();

    FMM< nDGridMap<FMCell, 2> > fmm;
    fmm.setEnvironment(&grid);
    fmm.setInitialAndGoalPoints(init_points, goal);
    fmm.compute();

    QImage output;
    {
        std::array<unsigned int,2> d = grid.getDimSizes();
        double max_val = grid.getMaxValue();
        CImg<double> img(d[0],d[1],1,1,0);
        // Filling the image flipping Y dim. We want now top left to be the (0,0).
        cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getValue()/max_val*255; }
        img.map( CImg<double>::jet_LUT256() );
        //img.display("", false);

        QImage imgQt(img.width(), img.height(), QImage::Format_ARGB32);

        cimg_forXY( img, x, y )
        {
           auto R = img( x, y, 0 );
           auto G = img( x, y, 1 );
           auto B = img( x, y, 2 );
           imgQt.setPixel( x, y, qRgb(R,G,B) );
        }

        output = imgQt;
    }

	std::vector <double> path_velocity; // Velocities profile
	GradientDescent< nDGridMap<FMCell, 2> > grad;	
	grid.coord2idx(goal, goal_idx);
	grad.apply(grid, goal_idx, path, path_velocity);

    Q_UNUSED(grad);

	// Fix path's 'y'
	for (auto & p : path) p[1] = (input.height() - 1) - p[1];

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
