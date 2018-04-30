#include <osgview.h>


OsgView::OsgView(QWidget* parent, const QGLWidget* shareWidget, WindowFlags f): QGLWidget(parent, shareWidget, f) , osgViewer::Viewer()
{
	_gw = new osgViewer::GraphicsWindowEmbedded(0,0,parent->width(),parent->height());
	setParent(parent);
	setFocusPolicy(Qt::StrongFocus);
	resize(parent->width(), parent->height());
	osg::DisplaySettings::instance()->setNumMultiSamples( 4 );
	getCamera()->setViewport(new osg::Viewport(0,0,parent->width(),parent->height()));
	getCamera()->setGraphicsContext(getGraphicsWindow());

	getCamera()->setProjectionMatrixAsPerspective(45.0f, static_cast<double>(parent->width())/static_cast<double>(parent->height()), 0.000001, 100000.0);
	getCamera()->setClearColor(osg::Vec4(0.,0.,0.,1.));
	setThreadingModel(osgViewer::Viewer::SingleThreaded);
	osg::DisplaySettings::instance()->setNumMultiSamples( 4 );
	init();
// 	osgGA::TrackballManipulator *tBall = new osgGA::TrackballManipulator;
// 	tBall->setHomePosition(osg::Vec3(0,0,0),osg::Vec3(0.f,0.,-40.),osg::Vec3(0.0f,1.f,0.0f), false);
// 	setCameraManipulator( tBall );
// 	osgw->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);

	show();
}

void OsgView::setHomePosition(osg::Vec3 eye, osg::Vec3 center, osg::Vec3 up, bool autoComputeHomePosition)
{
	osgGA::TrackballManipulator *tBall = new osgGA::TrackballManipulator;
	tBall->setHomePosition (eye,center,up,autoComputeHomePosition);
	setCameraManipulator( tBall );
}

OsgView::~OsgView()
{
}


void OsgView::init( )
{
	//Build basic scene
	root = new osg::Group();

 	//global stateset
	osg::StateSet *globalStateSet = new osg::StateSet;
	globalStateSet->setGlobalDefaults();
	globalStateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

	// enable lighting
// 	globalStateSet->setMode(GL_LIGHTING, osg::StateAttribute::ON);


	addEventHandler(new osgViewer::WindowSizeHandler);

	setSceneData( root.get() ) ;

	connect(&timer, SIGNAL(timeout()), this, SLOT(updateGL()));
	timer.start(30);
}

void OsgView::paintGL()
{
	//This calls are commented to avoid auto-call to the OSG render process.
// 	frame();
	//printFPS();
}



void OsgView::pickObject( const QPoint & p)
{

//     qDebug() << "Entering pickObject";
	// 0. Normalize mouse coordinates to -1,1 range.
	//For y, -1 would be the bottom of the window. 0.0 would be the middle of the window. +1 would be the top of the window.
	//For x, -1 would be the left hand side of the window. 0.0 would be the middle of the window. +1 would be the right hand side of the window.
	// 	QList<QPair<QPointF,QPointF> > listInt;
	// 	listInt.append(qMakePair( QPointF( 0.f, this->width() ), QPointF(-1.f,1.f)));
	// 	listInt.append(qMakePair( QPointF( 0.f, this->height() ), QPointF(-1.f, 1.f)));
	// 	QMat mt = QMat::afinTransformFromIntervals( listInt );
	// 	QVec pn = mt * QVec::homogeneousCoordinates(p);

  // 1. Create either a PolytopeIntersector, or a LineIntersector using the normalized mouse< co-ordinates
	osg::ref_ptr<osgUtil::LineSegmentIntersector> picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, p.x(),-p.y() + height());

  // 2. Create an IntersectionVisitor, passing the Intersector as parameter to the constructor.
	osgUtil::IntersectionVisitor IntersectionVisitor(picker);

  // 3. Launch the IntersectionVisitor on the root node of the scene graph. In an OSGART application, we ca launch it on the osgART::Scene node.
	this->getCamera()->accept(IntersectionVisitor);

  // 4. If the Intersector contains any intersections obtain the NodePath and search it to find the Node of interest.
	 osg::Vec3d xc;
    if ( picker->containsIntersections())
	 {
		osg::NodePath nP = picker->getFirstIntersection().nodePath;

		// Here we search the NodePath for the node of interest. This is where we can make use of our node naming.
		//qDebug() << "Number of nodes to root " << nP.size();
		for (uint i = 0; i < nP.size(); i++)
		{
			{
			  xc = picker->getFirstIntersection().getWorldIntersectPoint();
			  osg::Vec3 ret =  xc;
				ret[2] *= -1;
			  emit newWorld3DCoor(ret);
			  break;
			}
		}
	  }
}


void OsgView::printFPS( )
{
	static int fps=0;
	static QTime ti;
	if ((++fps % 50) == 0)
	{
		std::cout << "fps " << 50000 / ti.restart() << std::endl;
	}
}

void OsgView::resizeGL( int width, int height )
{
    _gw->getEventQueue()->windowResize(0, 0, width, height );
    _gw->resized(0,0,width,height);
}

void OsgView::keyPressEvent( QKeyEvent* event )
{
//	qDebug()<<"key pressed"<<event->text();
	emit keyPress(event->text());
    _gw->getEventQueue()->keyPress( (osgGA::GUIEventAdapter::KeySymbol) *(event->text().toStdString().c_str() ) );
}

void OsgView::keyReleaseEvent( QKeyEvent* event )
{
	emit keyRelease(event->text());
    _gw->getEventQueue()->keyRelease( (osgGA::GUIEventAdapter::KeySymbol) *(event->text().toStdString().c_str() ) );
}

void OsgView::mousePressEvent( QMouseEvent* event )
{
    int button = 0;
    switch(event->button())
    {
        case(Qt::LeftButton): button = 1; this->pickObject( QPoint(event->x(), event->y()) ); break;
        case(Qt::MidButton): button = 2; break;
        case(Qt::RightButton): button = 3; break;
        case(Qt::NoButton): button = 0; break;
        default: button = 0; break;
    }
    if (event->button() == Qt::LeftButton)
		emit newLeftCoor( QPointF(event->pos()) );
	else
		emit newRightCoor( QPointF(event->pos()) );
    _gw->getEventQueue()->mouseButtonPress(event->x(), event->y(), button);
}

void OsgView::mouseReleaseEvent( QMouseEvent* event )
{
    int button = 0;
    switch(event->button())
    {
        case(Qt::LeftButton): button = 1; break;
        case(Qt::MidButton): button = 2; break;
        case(Qt::RightButton): button = 3; break;
        case(Qt::NoButton): button = 0; break;
        default: button = 0; break;
    }
    emit endCoor(QPointF(event->pos()));
    _gw->getEventQueue()->mouseButtonRelease(event->x(), event->y(), button);
}

void OsgView::mouseMoveEvent( QMouseEvent* event )
{
    _gw->getEventQueue()->mouseMotion(event->x(), event->y());
}


