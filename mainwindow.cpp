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

long mymap(long x, long in_min, long in_max, long out_min, long out_max)
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

	uchar r = (uchar)mymap(v,prev_k,next_k,prev.r,next.r);
	uchar g = (uchar)mymap(v,prev_k,next_k,prev.g,next.g);
	uchar b = (uchar)mymap(v,prev_k,next_k,prev.b,next.b);
	return rgb_color(r,g,b);
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
	for(int i = 0; i <= ceil(255/10); i++)
		pointsByLevel.push_back(std::vector<point>());
	setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
	ui->setupUi(this);
	show();
  paleta = loadPaleta();
	cv_image = cv::Mat(480, 640, CV_8UC3);
	osgImage = NULL;
	rgb = new uint8_t[640*480*3];
	bb = new uint8_t[640*480*3];
	bf = new float[640*480*3];
	tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes36h11);
	init3D();

	cloud = NULL;
	viewer = new Viewer();
	// std::thread update (viewer->update);
	// viewer = new pcl::visualization::CloudViewer("cloud");

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
	luzblanca = new Luz(osgw, 0, osg::Vec4(0.0,0.0,0.0,1.0), osg::Vec4(1.0,1.0,1.0,1.0), osg::Vec4(1.0,1.0,1.0,1.0), osg::Vec4(0,0,0,1.0));
	luzroja = new Luz(osgw, 1, osg::Vec4(0.0,0.0,0.0,1.0), osg::Vec4(1.0,0.,0.,1.0), osg::Vec4(1.0,0.,0.,1.0), osg::Vec4(0,0,0,1.0));
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
	processTags();
	viewer->update();
}

#ifdef READ_DATA_FROM_DEVICE
void MainWindow::computeRGBD(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_l)
{
	static uint32_t count = 0;
	if (cloud == NULL)
		cloud = pcl::PointCloud< pcl::PointXYZRGBA >::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud_l));
	*cloud = *cloud_l;
	viewer->addPointCloud(cloud, "cloud", 0, 0, 0);
	// viewer->update();
	// viewer->showCloud(cloud_l);
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

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (std::string("../clouds/data")+std::to_string(ui->spinBox->value())+std::string(".pcd"), *cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file pcd: %s\n", (std::string("../clouds/data")+std::to_string(ui->spinBox->value())+std::string(".pcd").c_str()));
		exit(1);
	}
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
	std::vector< int > index;
	removeNaNFromPointCloud(*cloud, *cloud, index);
	int n;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cluster_clouds = euclideanClustering(cloud,n);
	int i=0;
	viewer->removeAllShapes();
	for(auto obj_scene:cluster_clouds)
	{
		i++;
		// pcl::PointCloud<pcl::PointXYZ>::Ptr *Obj = new ObjectType;
		// objectspointer.push_back(Obj);
		std::cout<<obj_scene->points.size()<<std::endl;
		// getBoundingBox(obj_scene,min_x, max_x, min_y, max_y, min_z, max_z);
		viewer->drawBoundingBox(obj_scene, std::to_string(i));

		// addCube(viewer, min_x, max_x, min_y, max_y, min_z, max_z, std::to_string(i));
	}
}

void MainWindow::processTags()
{

	//Apagar luces
	luzroja->switchLuz(false);
	luzblanca->switchLuz(false);
  	//Procesar AprilTags
	Mat imgray;
	cv::cvtColor(cv_image, imgray, cv::COLOR_RGB2GRAY);
	float k=1.3;
	vector<::AprilTags::TagDetection> detections = tagDetector->extractTags(imgray);
	//std::cout<<imgray.rows<<" "<<imgray.cols<<std::endl;
	//std::cout<<detections.size()<<std::endl;
	for(auto april: detections)
	{
		//std::cout<<"Detectado april "<<april.id<<std::endl;
		//std::cout<<april.cxy.first<<"----"<<april.cxy.second<<std::endl;
		// float x = mymap(april.cxy.first,0., 640., 0., -10.24*k/2);
		// float y = mymap(april.cxy.second,0., 480., 0., 7.68*k/2);
		// std::cout<<x<<"----"<<y<<std::endl;
		if(april.id == 31)
		{
			luzroja->move(-10.24*k/2, 7.68*k/2, -100); //POSICION HARDCODEADA A MITAD DEL MAPA , probar valores "mapeados"
			// luzroja->move(x, y, -100);
			luzroja->switchLuz(true);
		}
		else if(april.id == 30)
		{
			luzblanca->move(-10.24*k/2, 7.68*k/2, -100);
			// luzblanca->move(x, y, -100);
			luzblanca->switchLuz(true);
		}
	}
}

Mat FILTER(Mat input)
{
	for (int r=0; r<480; r++)
	{
		for (int c=0; c<640; c++)
		{
				if(input.at<uchar>(r, c) != 0  && input.at<uchar>(r+1, c) != 0 && input.at<uchar>(r, c+1) != 0 && input.at<uchar>(r+1, c+1) != 0 )
				{
					input.at<uchar>(r, c) = 0;
				}
		}
	}
	return input;
}

void MainWindow::searchWay(point src, point dst)
{
	std::cout << __FUNCTION__ << '\n';
	std::vector<point> pointsToWay;
	for(int i = src.getLevel(); i<=dst.getLevel(); i++)
	{
		pointsToWay.insert(pointsToWay.begin(), pointsByLevel[i].begin(), pointsByLevel[i].end());
	}
	Mat imgWay = cv::Mat::zeros(480, 640, CV_8U);
	Mat outimg = cv::Mat::zeros(480, 640, CV_8U);;
	for(auto point : pointsToWay)
	{
		imgWay.at<uchar>(point.x,point.y)=255;
	}
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( imgWay, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	for( unsigned int i = 0; i< contours.size(); i++ )
	{
		drawContours( outimg, contours, i, 255, 1, 8, hierarchy, 0, Point() );
	}
	line(outimg, Point( src.y, src.x), Point(dst.y,dst.x),255);
	imshow("outimg", outimg);
}

void MainWindow::drawColors()
{
	float minP = -1;
	float maxP = -1;
	// float min = -1;
	// float max = -1;
	cv::Mat mask = cv::Mat::zeros(480, 640, CV_8U);
  cv::Mat out = cv::Mat::zeros(480, 640, CV_8U);
	int erosion_size = 2;
	int erosion_dilate = 3;
	Mat element_erosion = getStructuringElement(cv::MORPH_RECT,
	      cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
	      cv::Point(erosion_size, erosion_size) );
	Mat element_dilate = getStructuringElement(cv::MORPH_RECT,
				cv::Size(2 * erosion_dilate + 1, 2 * erosion_dilate + 1),
				cv::Point(erosion_dilate, erosion_dilate) );
	for (uint32_t r=0; r<480; r++)
	{
		for (uint32_t c=0; c<640; c++)
		{
			const uint32_t idx = r*640+c;
			float v = bf[idx];

			if (std::isnan(v)){
				bb[idx*3 + 0] = bb[idx*3 + 1] = bb[idx*3 + 2] = 0;
				continue;
			}
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
				if ( r == 100 && c == 200)
					std::cout<< r << " " << c << " " << v <<std::endl;
				else if( r == 300 && c == 500)
					std::cout<< r << " " << c << " " << v <<std::endl;
				if((int)v%10 == 0)
				{
					mask.at<uchar>((int)r,(int)c) = 255;
					// return rgb_color(0,0,0);
				}
				int level = ceil(v/10);
				pointsByLevel.at(level-1).push_back(point((int)r,(int)c,level-1));
				rgb_color c = ucharize(v);
				bb[idx*3 + 0] = (uint8_t)c.r;
				bb[idx*3 + 1] = (uint8_t)c.g;
				bb[idx*3 + 2] = (uint8_t)c.b;

			}
		}
	}
	out = FILTER(mask);

	for (int r=0; r<480; r++)
	{
		for (int c=0; c<640; c++)
		{
				const uint32_t idx = r*640+c;
				if(out.at<uchar>(r, c) == 255 )
				{
					bb[idx*3 + 0] = (uint8_t)0;
					bb[idx*3 + 1] = (uint8_t)0;
					bb[idx*3 + 2] = (uint8_t)0;
				}
		}
	}

	searchWay(point(100, 200, ceil(115.114/10)), point(300, 500, ceil(116.571/10)));
}
