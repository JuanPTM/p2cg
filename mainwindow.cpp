#include <mutex>
#include <thread>

#include "mainwindow.h"



MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
	pointsByLevel = std::vector<std::vector<point>>(ceil(255/10.)+1,std::vector<point>());
	setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
	ui->setupUi(this);
	show();
  paleta = loadPaleta();
	cv_image = cv::Mat(480, 640, CV_8UC3);
	osgImage = NULL;
	rgb = new uint8_t[640*480*3];
	rgb_to_process = new uint8_t[640*480*3];
	bb = new uint8_t[640*480*3];
	bb_to_process = new uint8_t[640*480*3];
	bf = new float[640*480*3];
	bf_to_process = new float[640*480*3];
	tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes36h11);
	init3D();

	cloud = NULL;
	cloud_to_process = NULL;
	viewer = new Viewer();
	// std::thread update (viewer->update);
	// viewer = new pcl::visualization::CloudViewer("cloud");

	connect(&timerOSG, SIGNAL(timeout()), this, SLOT(computeOSG()));
	connect(&timerSearchWay, SIGNAL(timeout()), this, SLOT(button_slot()));

	timerSearchWay.start(15000);
	timerOSG.start(2);
	connect(ui->loadButton, SIGNAL(clicked()), this, SLOT(button_slot()));
	initCamera();
	// TprocessScene = std::thread(&MainWindow::processScene, this);
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
			// grabber = new pcl::OpenNIGrabber();

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
	geom = new osg::Geometry();
	geom->setUseDisplayList(false);

	osg::Geode* shapeGeode = new osg::Geode();
	shapeGeode->addDrawable(geom.get());
	pat->addChild(shapeGeode);


	// float k = 1.3;
	osg::Box *box = new osg::Box( osg::Vec3(0, 0, 0), -10.24*Kfactor, 7.68*Kfactor, 0.01);
	osg::ShapeDrawable *boxDrawable = new osg::ShapeDrawable(box);
	shapeGeode->addDrawable(boxDrawable);

	osgImage = new osg::Image;
	osgTexture = new osg::Texture2D;
	bStateSetIMAGEN = shapeGeode->getOrCreateStateSet();
	bStateSetIMAGEN->setTextureMode(0, GL_TEXTURE_GEN_R, osg::StateAttribute::ON);
	luzblanca = new Luz(osgw, 0, osg::Vec4(0.0,0.0,0.0,1.0), osg::Vec4(1.0,1.0,1.0,1.0), osg::Vec4(1.0,1.0,1.0,1.0), osg::Vec4(0,0,0,1.0));
	luzroja = new Luz(osgw, 1, osg::Vec4(0.0,0.0,0.0,1.0), osg::Vec4(1.0,0.,0.,1.0), osg::Vec4(1.0,0.,0.,1.0), osg::Vec4(0,0,0,1.0));



	osgway = new osgWay(osgw);

}


void MainWindow::computeOSG()
{
	memcpy(cv_image.data, rgb, 640*480*3);
	cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
	imshow("Image", cv_image);
	cvWaitKey(1);

	mtx_bb.lock();
	osgImage->setImage(640, 480, 1, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, bb, osg::Image::NO_DELETE);
	mtx_bb.unlock();
	osgTexture->setImage(0, osgImage);
	bStateSetIMAGEN->setTextureAttributeAndModes(0, osgTexture, osg::StateAttribute::ON);
	osgw->frame();
	processTags();
	osgway->update();
}

#ifdef READ_DATA_FROM_DEVICE
void MainWindow::computeRGBD(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_l)
{
	std::vector< int > index;
	// removeNaNFromPointCloud(*cloud_l, *cloud_l, index);
	mtx_cloud.lock();
	if (cloud == NULL)
		cloud = pcl::PointCloud< pcl::PointXYZRGBA >::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud_l));
	else
		*cloud = *cloud_l;
	removeNaNFromPointCloud(*cloud, *cloud, index);
	mtx_cloud.unlock();
	viewer->addPointCloud(cloud, "cloud", 0, 0, 0);
	viewer->update();
}

void MainWindow::computeImages(const boost::shared_ptr<pcl::io::openni2::Image> &o_image)
{
	static unsigned count = 0;
	count++;

	if (osgImage == NULL)
		return;
	mtx_rgb.lock();
	o_image->fillRGB(o_image->getWidth(), o_image->getHeight(), rgb, 0);
	mtx_rgb.unlock();
}

void MainWindow::computeImages2(const boost::shared_ptr<pcl::io::openni2::DepthImage> &dimg)
{
	static uint32_t count = 0;
	count++;

	if (osgImage == NULL)
		return;
	mtx_bf.lock();
	dimg->fillDepthImage(640, 480, bf);
	mtx_bf.unlock();
	// drawColors();
}

void MainWindow::processScene()
{
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cluster_clouds;
	int y,x,n;
	while (true) {
		if(cloud==NULL)
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
			continue;
		}
		// mtx_exclude.lock();

		mtx_cloud.lock();
		mtx_bf.lock();
		mtx_rgb.lock();
		std::swap(cloud, cloud_to_process);
		std::swap(bf, bf_to_process);
		std::swap(rgb, rgb_to_process);
		mtx_rgb.unlock();
		mtx_bf.unlock();
		mtx_cloud.unlock();
		// Eliminamos los puntos del arenero y de el suelo
		drawColors();
		// mtx_exclude.unlock();

	}
}

#else
void MainWindow::readPCD()
{
	if (cloud == NULL)
		cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (osgImage == NULL)
		return;

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (std::string("../clouds/data")+std::to_string(ui->spinBox->value())+std::string(".pcd"), *cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file pcd: %s\n", (std::string("../clouds/data")+std::to_string(ui->spinBox->value())+std::string(".pcd").c_str()));
		exit(1);
	}
	// 0.805 1.007

	cloud = Filter_in_axis(cloud, "z", 0.0, 1.05, false);


	viewer->addPointCloud(cloud, "cloud", 0, 0, 0);
	// viewer->showCloud(cloud);

}

void MainWindow::readDepth()
{
	if (osgImage == NULL)
		return;

	printf("depth\n");
	readBufferFromFile(std::string("../clouds/data")+std::to_string(ui->spinBox->value())+std::string(".depth"), bf, 640*480*4);
	readBufferFromFile(std::string("../clouds/data")+std::to_string(ui->spinBox->value())+std::string(".rgb"), rgb, 640*480*3);
	// osgTexture

}
#endif

void MainWindow::button_slot()
{
	// move(0,0);
	// mtx_exclude.lock();
	int n;
#ifdef READ_DATA_FROM_DEVICE
	n = ui->spinBox->value();
	pcl::io::savePCDFileASCII (std::string("data")+std::to_string(n)+std::string(".pcd"), *cloud);
	writeBufferToFile(std::string("data")+std::to_string(n)+std::string(".depth"), bf, 640*480*4);
	writeBufferToFile(std::string("data")+std::to_string(n)+std::string(".rgb"), rgb, 640*480*3);
#else
	readPCD();
	readDepth();
#endif

	mtx_cloud.lock();
	mtx_bf.lock();
	mtx_rgb.lock();
	std::swap(cloud, cloud_to_process);
	std::swap(bf, bf_to_process);
	std::swap(rgb, rgb_to_process);
	mtx_rgb.unlock();
	mtx_bf.unlock();
	mtx_cloud.unlock();
	auto cloud_for_euclideanClustering = Filter_in_axis(cloud_to_process, "z", 0.0, 0.8, false);
	if (cloud_for_euclideanClustering->points.size() !=0)
	{
		// viewer->addPointCloud(cloud_for_euclideanClustering, "cloud_for_euclideanClustering", 255, 0, 0);
		std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cluster_clouds = euclideanClustering(cloud_for_euclideanClustering,n);
		int i=0;
		viewer->removeAllShapes();
		srcDstWay.clear();
		for(auto obj_scene:cluster_clouds)
		{
			i++;
			if(srcDstWay.size()<2)
			{
				auto center = getCentroid(obj_scene);
				// Distancia focal 485pixel
				float f = 485;
				int y = f*center.x/center.z+640/2;
				int x = f*center.y/center.z+480/2;
				srcDstWay.push_back(point(x,y,0));
				// viewer->addCoordinateSystem(center.x, center.y, center.z, std::to_string(i));
				// viewer->drawBoundingBox(obj_scene, std::to_string(i));
			}
		}
	}
	drawColors();
	// mtx_exclude.unlock();

}

void MainWindow::processTags()
{

	//Apagar luces
	luzroja->switchLuz(false);
	// float k=1.3;
	luzblanca->move(-10.24*Kfactor/2, 7.68*Kfactor/2, -100);
	luzblanca->switchLuz(true);
  	//Procesar AprilTags
	Mat imgray;
	cv::cvtColor(cv_image, imgray, cv::COLOR_RGB2GRAY);
	vector<::AprilTags::TagDetection> detections = tagDetector->extractTags(imgray);

	for(auto april: detections)
	{
		int x = april.cxy.first;
		int y = april.cxy.second;
		if(april.id == 31)
		{
			luzroja->move(x*Kfactor/100, y*Kfactor/100, -100);
			luzroja->switchLuz(true);
		}
		else if(april.id == 30)
		{
			luzblanca->move(x*Kfactor/100, y*Kfactor/100, -100);
			luzblanca->switchLuz(true);
		}
	}
}

void MainWindow::drawColors()
{
	point src,dst;
	float minP = -1, maxP = -1;
	cv::Mat mask = cv::Mat::zeros(480, 640, CV_8U);
	std::vector<float> disSrcDst(2,std::numeric_limits<float>::max());
	uint32_t idx;
	int level;

	for(auto listLevel:pointsByLevel)
	listLevel.clear();

	for (uint32_t r=0; r<480; r++)
	{
		for (uint32_t c=0; c<640; c++)
		{
			idx = r*640+c;
			float v = bf_to_process[idx];

			if (std::isnan(v)){
				bb_to_process[idx*3 + 0] = bb_to_process[idx*3 + 1] = bb_to_process[idx*3 + 2] = 0;
				continue;
			}
			if (v>maxP)
			maxP = v;
			if (v<minP or minP<0)
			minP = v;
			if (v<minV)
			{
				bb_to_process[idx*3 + 0] = bb_to_process[idx*3 + 1] = bb_to_process[idx*3 + 2] = 0;
			}
			else
			{
				v = (v-minV)/(maxV-minV)*255;
				normalizeFloat(v);
				if((int)v%10 == 0)
				{
					mask.at<uchar>((int)r,(int)c) = 255;
				}
				level = ceil(v/10.);
				point p = point((int)r,(int)c,level);
				if (srcDstWay.size() == 2)
				for(unsigned int i=0; i < 2; i++)
				{
					if(srcDstWay.at(i).adjacentDistance(p)<disSrcDst.at(i))
					{
						disSrcDst.at(i) = srcDstWay.at(i).adjacentDistance(p);
						if(i==1)
							src = p;
						else
							dst = p;
					}
				}
				pointsByLevel.at(level).push_back(p);
				rgb_color c = ucharize(v, paleta);
				bb_to_process[idx*3 + 0] = (uint8_t)c.r;
				bb_to_process[idx*3 + 1] = (uint8_t)c.g;
				bb_to_process[idx*3 + 2] = (uint8_t)c.b;
			}
		}
	}
	// out =
	FILTER(mask);

	for (int r=0; r<480; r++)
	{
		for (int c=0; c<640; c++)
		{
			idx = r*640+c;
			if(mask.at<uchar>(r, c) == 255 )
			{
				bb_to_process[idx*3 + 0] = bb_to_process[idx*3 + 1] = bb_to_process[idx*3 + 2] = (uint8_t)0;
			}
		}
	}
	mtx_bb.lock();
	std::swap(bb, bb_to_process);
	mtx_bb.unlock();
	// animar camino
	osgway->hide();
	if(srcDstWay.size()>=2)
	{
		auto way = searchWay(src, dst);
		osgway->setWay(way);
	}
}

std::vector<point> MainWindow::searchWay(point src, point dst)
{
	std::vector<point> pointsToWay;
	vector<int> way;
	long dstIndex,srcIndex;
	int min = std::min(src.getLevel(),dst.getLevel());
	int max = std::max(src.getLevel(),dst.getLevel());
	int value = 6;
	do {
		pointsToWay.clear();
		for(int i = min; i<=max; i++)
			pointsToWay.insert(pointsToWay.begin(), pointsByLevel[i].begin(), pointsByLevel[i].end());
		pointsToWay = reduceListPoints(pointsToWay,value);
		if(src.x%value!=0 || src.y%value!=0)
			pointsToWay.push_back(src);
		if(dst.x%value!=0 || dst.y%value !=0)
			pointsToWay.push_back(dst);
		if(min>0)
		 	min--;
		if(max<ceil(255./10)-1)
		 	max++;

		srcIndex = std::distance(pointsToWay.begin(), std::find(pointsToWay.begin(), pointsToWay.end(), src));
		dstIndex = std::distance(pointsToWay.begin(), std::find(pointsToWay.begin(), pointsToWay.end(), dst));

		way = dijk(srcIndex, dstIndex, pointsToWay,value);
	} while(way.empty() || way.back() != dstIndex || way.front() != srcIndex);
	std::vector<point> result;
	for(auto const &i:way)
		result.push_back(pointsToWay.at(i));
	return result;
/*
	 for(auto const &i:way)
	 {
	 	int curX = pointsToWay[i].x;
	 	int curY = pointsToWay[i].y;
	 	for (int x =curX-value/2; x<curX+value/2; x++)
	 		for (int y =curY-value/2; y<curY+value/2; y++)
	 		{
	 			if(x<0 || 480<=x || y<0 || 640<=y) continue;
	 			int idx = x*640+y;
	 			bb[idx*3 + 0] = bb[idx*3 + 1] = 0;
	 			bb[idx*3 + 2] = 255;
	 		}
	 }
	 int curX = src.x;
	 int curY = src.y;
	 for (int x =curX-value/2; x<curX+value/2; x++)
	 	for (int y =curY-value/2; y<curY+value/2; y++)
	 	{
	 		if(x<0 || 480<=x || y<0 || 640<=y) continue;
	 		int idx = x*640+y;
	 		bb[idx*3 + 1] = bb[idx*3 + 2] = 0;
	 		bb[idx*3 + 0] = 255;
	 	}
	 curX = dst.x;
	 curY = dst.y;
	 for (int x =curX-value/2; x<curX+value/2; x++)
	 	for (int y =curY-value/2; y<curY+value/2; y++)
	 	{
	 		if(x<0 || 480<=x || y<0 || 640<=y) continue;
	 		int idx = x*640+y;
	 		bb[idx*3 + 1] = bb[idx*3 + 2] = 0;
	 		bb[idx*3 + 0] = 255;
	 	}
*/
}
