#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#define READ_DATA_FROM_DEVICE

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
#include <limits>
#include "Luz/luz.h"
#include "ui_mainwindow.h"

#include "computepointcloud/computepointcloud.h"
#include "processfunction/processfunction.h"
#include "descriptors/descriptors.h"
#include "osgWay/osgWay.h"
#include "types/types.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>
#ifndef Q_MOC_RUN
	#include <pcl/visualization/pcl_visualizer.h>
#ifdef READ_DATA_FROM_DEVICE
	#include <pcl/io/openni2_grabber.h>
#endif
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
#include <osg/Texture3D>
#include <osgview.h>
#include <mutex>
#include <thread>
#include <algorithm>

using namespace cv;
using namespace computepointcloud;
using namespace processfunction;
namespace Ui
{
	class MainWindow;
}

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
	void processScene();
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
	boost::shared_ptr<DESCRIPTORS> descriptor_matcher;

	std::map<float,rgb_color> paleta;
	std::vector<point> srcDstWay;
	std::vector<std::vector<point>>pointsByLevel;
	// std::vector<point> searchWay(point src, point dst);
	osgWay *osgway;
	std::mutex mtx_cloud, mtx_rgb, mtx_bf, mtx_bb;
	std::thread TprocessScene;

	QTimer timerOSG;
	Ui::MainWindow *ui;
	pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud;
	pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud_to_process;
	Viewer *viewer;
	OsgView *osgw;
	::AprilTags::TagDetector* tagDetector;

	void processTags();

#ifdef READ_DATA_FROM_DEVICE
	pcl::Grabber* grabber;
	// pcl::io::OpenNI2Grabber* grabber;
#endif

	cv::Mat cv_image;
	uint8_t *rgb, *rgb_to_process;
	float *bf, *bf_to_process;
	uint8_t *bb, *bb_to_process;
	osg::Image *osgImage;
	osg::ref_ptr<osg::Geometry> geom;
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
