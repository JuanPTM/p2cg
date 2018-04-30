/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef OSGVIEW_H
#define OSGVIEW_H

/**
	@author Pablo Bustos and RoboLab's team
*/

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/GraphicsWindow>
#include <osgGA/TrackballManipulator>
#include <osgGA/FirstPersonManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <QtGui>
#include <QtOpenGL/QGLWidget>


#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Shape>
#include <osg/Point>
#include <osg/Material>
#include <osg/Quat>
#include <osg/Image>
#include <osg/ShapeDrawable>
#include <osg/Camera>
#include <osg/Geometry>
#include <osg/LineSegment>
#include <osg/TexMat>
#include <osgUtil/Tessellator>
#include <osgUtil/IntersectVisitor>
#include <osg/io_utils>




using Qt::WindowFlags;
// using namespace RMat;

class OsgView : public QGLWidget, public osgViewer::Viewer
{
Q_OBJECT
public:

	struct laserMeasure{
		float dist;
		float angle;
	};
	osg::Geode* TextGeode;
	osgText::Text* textOne;

	void setClearColor(osg::Vec3 color) { getCamera()->setClearColor(osg::Vec4(color[0], color[1], color[2], 1)); }
// 	QVector<laserMeasure> laserData;

	OsgView(QWidget * parent, const QGLWidget * shareWidget = 0, WindowFlags f = 0);
	~OsgView();
	virtual void paintGL();
	osgViewer::GraphicsWindow* getGraphicsWindow() { return _gw.get(); }
	const osgViewer::GraphicsWindow* getGraphicsWindow() const { return _gw.get(); }
	void addCamaraManipulator(int n, std::string name, osg::ref_ptr<osgGA::TrackballManipulator> man);
	void setScene( osg::ref_ptr<osg::Group> root ){ setSceneData( root.get() );};

	void setHomePosition (osg::Vec3 eye, osg::Vec3 center, osg::Vec3 up, bool autoComputeHomePosition);
	void init();



	osg::Group * getRootGroup()	{ return root; }

	void autoResize() {
		if (parentWidget()) {
			setFixedSize(parentWidget()->width(), parentWidget()->height());
		}
		getCamera()->setProjectionMatrixAsPerspective(70.0f, static_cast<double>(width())/static_cast<double>(height()), 0.00000001, 1000000.0);
	}

  void autoResize(float HFOV) {
    if (parentWidget()) {
      setFixedSize(parentWidget()->width(), parentWidget()->height());
    }
    getCamera()->setProjectionMatrixAsPerspective(HFOV, static_cast<double>(width())/static_cast<double>(height()), 0.00000001, 1000000.0);
  }

	//Picking objects
	void pickObject( const QPoint & p);


protected:
	QTimer timer;
	virtual void resizeGL( int width, int height );
	virtual void keyPressEvent( QKeyEvent* event );
	virtual void keyReleaseEvent( QKeyEvent* event );
	virtual void mousePressEvent( QMouseEvent* event );
	virtual void mouseReleaseEvent( QMouseEvent* event );
	virtual void mouseMoveEvent( QMouseEvent* event );
	osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _gw;
private:
	osg::ref_ptr<osg::Group> root;
	osg::ref_ptr<osg::Image> osgImage;

public:
	QMutex mutex;

	inline void printFPS();
signals:
	void newWorld3DCoor( const osg::Vec3 & vec);
	void newLeftCoor(QPointF p);
	void newRightCoor(QPointF p);
	void endCoor(QPointF p);
	void keyPress(QString t);
	void keyRelease(QString t);

};

#endif
