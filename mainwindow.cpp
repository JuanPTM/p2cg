#include <mutex>
#include <thread>

#include "mainwindow.h"

#include "ui_mainwindow.h"


float normalizeFloat(float v)
{
	if (v > 255)
		v = 255;
	if (v < 0)
		v = 0;
	return v;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//AQUI AÑADIR LA PALETA
rgb_color MainWindow::ucharize(float v)
{
	if (v >= 255)
		return rgb_color(0,0,80);
	if (v <= 0)
		return rgb_color(255,255,255);
	rgb_color prev(0,0,80), next(0,0,80);
	float prev_k = 255, next_k = 255;
	for (auto const& fila : paleta)
	{
		if (fila.first > v){
			next = fila.second;
			next_k = fila.first;
			break;
		}
		else{
			prev_k = fila.first;
			prev = fila.second;
		}
	}

	uchar r = (uchar)map(v,prev_k,next_k,prev.r,next.r);
	uchar g = (uchar)map(v,prev_k,next_k,prev.g,next.g);
	uchar b = (uchar)map(v,prev_k,next_k,prev.b,next.b);
	return rgb_color(r,g,b);
}


pcl::PointCloud< pcl::PointXYZRGBA >::Ptr copy_pointcloud(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud)
{
	pcl::PointCloud< pcl::PointXYZRGBA >::Ptr copy_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));
	return copy_cloud;
}

std::map<float,rgb_color> loadPaleta()
{
	std::string k,r,g,b;
	ifstream readFile("../paleta.cfg");
	std::string line;
	std::map<float,rgb_color> paleta;
	while(std::getline(readFile,line))   {
    std::stringstream iss(line);
    std::getline(iss, k, '=');
    std::getline(iss, r, ',');
    std::getline(iss, g, ',');
    std::getline(iss, b, ',');
		// std::cout<<std::stof(k)<<" "<<std::stoi(r)<<" "<<std::stoi(g)<<" "<<std::stoi(b)<<std::endl;
		paleta[std::stof(k)] = rgb_color(std::stoi(r),std::stoi(g),std::stoi(b));
	}
	readFile.close();
	return paleta;
}


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
	setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
	ui->setupUi(this);
	show();
  paleta = loadPaleta();
	cv_image = cv::Mat(480, 640, CV_8UC3);
	osgImage = NULL;
	rgb = new uint8_t[640*480*3];
	bb = new uint8_t[640*480*3];
	bf = new float[640*480*3];
	init3D();

	cloud = NULL;

	viewer = new pcl::visualization::CloudViewer("cloud");

	connect(&timerOSG, SIGNAL(timeout()), this, SLOT(computeOSG()));
	timerOSG.start(2);
	connect(ui->loadButton, SIGNAL(clicked()), this, SLOT(button_slot()));

	initCamera();
}

MainWindow::~MainWindow()
{
#ifdef READ_DATA_FROM_DEVICE
	grabber->stop();
#endif
	delete ui;
}

void MainWindow::initCamera()
{
#ifdef READ_DATA_FROM_DEVICE
	int tries = 0;

	while (tries < 3)
	{
		try
		{
			// create a new grabber for OpenNI devices
			grabber = new pcl::io::OpenNI2Grabber();

			// Register Callback Function
			boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&MainWindow::computeRGBD, this, _1);
			grabber->registerCallback(f);

			boost::function<void(const boost::shared_ptr<pcl::io::openni2::Image> &)> f2 = boost::bind(&MainWindow::computeImages, this, _1);
			grabber->registerCallback(f2);

			boost::function<void(const boost::shared_ptr<pcl::io::openni2::DepthImage> &)> f3 = boost::bind(&MainWindow::computeImages2, this, _1);
			grabber->registerCallback(f3);

			// Start Retrieve Data
			grabber->start();
			tries = 0;
			break;
		}
		catch(...)
		{
			fprintf(stderr, "Attempt %d failed.\n", tries);
			tries++;
			sleep(2);
		}
	}

	if (tries)
	{
		exit(-1);
	}
#endif
}

void MainWindow::init3D()
{
	/// osgview
	osgw = new OsgView(ui->widget);
	/// manipulator
	osgGA::TrackballManipulator *manipulator;
	manipulator = new osgGA::TrackballManipulator;
	osg::Vec3 eye( osg::Vec3d(  0,   0, -10) );
	osg::Vec3 front( osg::Vec3d(0,   0, 10) );
	osg::Vec3 up( osg::Vec3d(   0, -10,  0) );
	manipulator->setHomePosition(eye, front, up, false);
	osgw->setCameraManipulator(manipulator, true);

	auto pat = new osg::PositionAttitudeTransform;
	osgw->getRootGroup()->addChild(pat);
	osg::Geode* shapeGeode = new osg::Geode();
	pat->addChild(shapeGeode);


	float k = 1.3;
	osg::Box *box = new osg::Box( osg::Vec3(0, 0, 0), -10.24*k, 7.68*k, 0.01);
	osg::ShapeDrawable *boxDrawable = new osg::ShapeDrawable(box);
 	boxDrawable->setColor(osg::Vec4(1,1,1,1));
	shapeGeode->addDrawable(boxDrawable);

	osgImage = new osg::Image;
	osgTexture = new osg::Texture2D;
	bStateSetIMAGEN = shapeGeode->getOrCreateStateSet();
	bStateSetIMAGEN->setTextureMode(0, GL_TEXTURE_GEN_R, osg::StateAttribute::ON);

}


void MainWindow::computeOSG()
{
	memcpy(cv_image.data, rgb, 640*480*3);
	cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
	imshow("Image", cv_image);
	cvWaitKey(1);


	osgImage->setImage(640, 480, 1, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, bb, osg::Image::NO_DELETE);
	osgTexture->setImage(0, osgImage);
	bStateSetIMAGEN->setTextureAttributeAndModes(0, osgTexture, osg::StateAttribute::ON);
	osgw->frame();
}


#ifdef READ_DATA_FROM_DEVICE
void MainWindow::computeRGBD(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_l)
{
	static uint32_t count = 0;
	if (cloud == NULL)
		cloud = pcl::PointCloud< pcl::PointXYZRGBA >::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud_l));
	*cloud = *cloud_l;
	viewer->showCloud(cloud_l);
}


void MainWindow::computeImages(const boost::shared_ptr<pcl::io::openni2::Image> &o_image)
{
	static unsigned count = 0;
	count++;

	if (osgImage == NULL)
		return;

	o_image->fillRGB(o_image->getWidth(), o_image->getHeight(), rgb, 0);
}

void MainWindow::computeImages2(const boost::shared_ptr<pcl::io::openni2::DepthImage> &dimg)
{
	static uint32_t count = 0;
	count++;

	if (osgImage == NULL)
		return;

	dimg->fillDepthImage(640, 480, bf);

	drawColors();
}


#else
void MainWindow::readPCD()
{
	if (cloud == NULL)
		cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (osgImage == NULL)
		return;

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (std::string("../data")+std::to_string(ui->spinBox->value())+std::string(".pcd"), *cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file pcd: %s\n", (std::string("../data")+std::to_string(ui->spinBox->value())+std::string(".pcd").c_str()));
		exit(1);
	}
	viewer->showCloud(cloud);

}
void MainWindow::readDepth()
{
	if (osgImage == NULL)
		return;

	printf("depth\n");
	readBufferFromFile(std::string("../data")+std::to_string(ui->spinBox->value())+std::string(".depth"), bf, 640*480*4);
	readBufferFromFile(std::string("../data")+std::to_string(ui->spinBox->value())+std::string(".rgb"), rgb, 640*480*3);
	drawColors();
}
#endif


void MainWindow::button_slot()
{
	move(0,0);
#ifdef READ_DATA_FROM_DEVICE
	int n = ui->spinBox->value();
	pcl::io::savePCDFileASCII (std::string("data")+std::to_string(n)+std::string(".pcd"), *cloud);
	writeBufferToFile(std::string("data")+std::to_string(n)+std::string(".depth"), bf, 640*480*4);
	writeBufferToFile(std::string("data")+std::to_string(n)+std::string(".rgb"), rgb, 640*480*3);
#else
	readPCD();
	readDepth();
#endif
}

void MainWindow::drawColors()
{
	float minP = -1;
	float maxP = -1;
	float min = -1;
	float max = -1;

	for (uint32_t r=0; r<480; r++)
	{
		for (uint32_t c=0; c<640; c++)
		{
			const uint32_t idx = r*640+c;
			float v = bf[idx];

			if (std::isnan(v))
				continue;
			if (v>maxP)
				maxP = v;
			if (v<minP or minP<0)
				minP = v;

			float minV = 0.75;
			float maxV = 1.1;


			if (v<minV)
			{
				bb[idx*3 + 0] = bb[idx*3 + 1] = bb[idx*3 + 2] = 0;
			}
			else
			{
				v -= minV;
				v /= maxV-minV;
				v = normalizeFloat(v*255);
				rgb_color c = ucharize(v);
				bb[idx*3 + 0] = (uint8_t)c.r;
				bb[idx*3 + 1] = (uint8_t)c.g;
				bb[idx*3 + 2] = (uint8_t)c.b;
			}
		}
	}
}
