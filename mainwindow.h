#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

// #define READ_DATA_FROM_DEVICE

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include "Luz/luz.h"
#include "computepointcloud/computepointcloud.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>
#ifndef Q_MOC_RUN
	#include <pcl/visualization/pcl_visualizer.h>
	// #include <pcl/io/openni2_grabber.h>
	#include <pcl/visualization/boost.h>

	#include <opencv2/core/core.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/features2d/features2d.hpp>
	#include <opencv2/calib3d/calib3d.hpp>
	#include "viewer/viewer.h"
	#include <pcl/visualization/cloud_viewer.h>
	#include <pcl/io/pcd_io.h>
#endif

#include <QMainWindow>
#include <QtCore>

#include <QGraphicsPixmapItem>

#include <osgview.h>
using namespace cv;
using namespace computepointcloud;

namespace Ui
{
	class MainWindow;
}

typedef struct rgb_color
{
	uchar r;
	uchar g;
	uchar b;
	public:
	rgb_color(){
		r=255;
		g=255;
		b=255;
	}
	rgb_color(uchar r_, uchar g_, uchar b_){
		r=r_;
		g=g_;
		b=b_;
	};
} rgb_color;
typedef struct point
{
	int x;
	int y;
	int level;
	public:
	point(){
		x=0;
		y=0;
		level = -1;
	}
	point(int x_, int y_){
		x=x_;
		y=y_;
		level = -1;
	};
	point(int x_, int y_, int level_){
		x=x_;
		y=y_;
		level = level_;
	};
	bool operator==(const point& other) const
  {
		return (x==other.x && y==other.y);
	};
	bool adjacent(const point& other) const
	{
		return (abs(x-other.x) == 1 && abs(y-other.y) == 1);
	};
	void setLevel(int level_)
	{
		level=level_;
	};
	int getLevel()
	{
		return level;
	};
} point;



class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

#ifdef READ_DATA_FROM_DEVICE
	void computeRGBD(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void computeImages(const boost::shared_ptr<pcl::io::openni2::Image> &);
	void computeImages2(const boost::shared_ptr<pcl::io::openni2::DepthImage>&);
#else
	void readPCD();
	void readDepth();
#endif

private slots:
	void computeOSG();

	void button_slot();
	void drawColors();

	void initCamera();
	void init3D();


private:
	std::map<float,rgb_color> paleta;
	std::vector<std::vector<point>>pointsByLevel;
	void searchWay(point src, point dst);
	void processTags();
	rgb_color ucharize(float v);

	QTimer timerOSG;
	Ui::MainWindow *ui;
	pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud;
	// pcl::visualization::CloudViewer *viewer;
	Viewer *viewer;
	OsgView *osgw;
	::AprilTags::TagDetector* tagDetector;

#ifdef READ_DATA_FROM_DEVICE
	pcl::io::OpenNI2Grabber* grabber;
#endif

	cv::Mat cv_image;
	uint8_t *rgb;
	float *bf;
	uint8_t *bb;
	osg::Image *osgImage;
	osg::Texture *osgTexture;
	osg::StateSet *bStateSetIMAGEN;
	Luz *luzblanca, *luzroja;

	static constexpr float minV = 0.75;
	static constexpr float maxV = 1.1;
	static constexpr float snowLvl = 60;
	static constexpr float waterLvl = 130;


	void writeBufferToFile(std::string str, void *buff, uint32_t nbytes)
	{
		auto fd = fopen(str.c_str(), "w");
		fwrite(buff, 1, nbytes, fd);
		fclose(fd);
	}
	void readBufferFromFile(std::string str, void *buff, uint32_t nbytes)
	{
		auto fd = fopen(str.c_str(), "r");
		fread(buff, 1, nbytes, fd);
		fclose(fd);
	}
};

#endif // MAINWINDOW_H
