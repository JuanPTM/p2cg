
#include "luz.h"

Luz::Luz(OsgView *osgw,int lightnum, osg::Vec4 pos, osg::Vec4 specular, osg::Vec4 diffuse, osg::Vec4 ambient)
{
  parent = osgw;
  light = new osg::Light;
  light->setLightNum(lightnum);
  light->setPosition( pos);
  light->setSpecular( specular);
  light->setDiffuse(  diffuse);
  light->setAmbient( ambient);
  osg::ref_ptr<osg::LightSource> lightSource1 = new osg::LightSource;
  lightSource1->setLight(light);
  this->setPosition(osg::Vec3(0.0f,0.0f,0.0f));
  this->addChild(lightSource1);
  osgw->getRootGroup()->addChild(this);
}

Luz::~Luz()
{
  // delete light;
}

void Luz::move(float x, float y, float z)
{
  this->setPosition(osg::Vec3(x,y,z));
}

void Luz::switchLuz(bool state)
{
  if(state)
    parent->getRootGroup()->getOrCreateStateSet()->setMode((int)GL_LIGHT0 + light->getLightNum(), osg::StateAttribute::ON);
  else
    parent->getRootGroup()->getOrCreateStateSet()->setMode((int)GL_LIGHT0 + light->getLightNum(), osg::StateAttribute::OFF);

}
