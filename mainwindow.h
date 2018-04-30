#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

// #define READ_DATA_FROM_DEVICE

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#ifndef Q_MOC_RUN
	#include <pcl/visualization/pcl_visualizer.h>
	// #include <pcl/io/openni2_grabber.h>
	#include <pcl/visualization/boost.h>

	#include <opencv2/core/core.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/features2d/features2d.hpp>
	#include <opencv2/calib3d/calib3d.hpp>
	#include <pcl/visualization/cloud_viewer.h>
	#include <pcl/io/pcd_io.h>
#endif

#include <QMainWindow>
#include <QtCore>

#include <QGraphicsPixmapItem>

#include <osgview.h>

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
	QTimer timerOSG;
	Ui::MainWindow *ui;
	pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud;
	pcl::visualization::CloudViewer *viewer;
	OsgView *osgw;

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
