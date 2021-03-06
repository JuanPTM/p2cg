
#include "osgWay.h"


osgWay::osgWay(OsgView *osgw)
{
  // float k=1.3;
  osg::Box *box = new osg::Box(osg::Vec3(-10.24*Kfactor/2,-7.68*Kfactor/2,0.0f),0.1,0.1,0.1);
  osg::ShapeDrawable *boxDrawable = new osg::ShapeDrawable(box);
  boxDrawable->setColor(osg::Vec4(1.0,0.0,0.0,1.0));
  osg::Geode *geo = new osg::Geode();
  geo->addDrawable(boxDrawable);

  for(int i=0; i<MAX_POINTS; i++)
  {
    osg::PositionAttitudeTransform *pat = new osg::PositionAttitudeTransform();
    pat->addChild(geo);
    this->addChild(pat);
  }
  numActivePats =0;
  osgw->getRootGroup()->addChild(this);
  this->setAllChildrenOff();
  step=4;
  state=0;
  osg = osgw;
  osg::PositionAttitudeTransform *patsrc = new osg::PositionAttitudeTransform();
  osg::PositionAttitudeTransform *patdst = new osg::PositionAttitudeTransform();
  osg::Node *osgMeshSRC = osgDB::readNodeFile("../mesh/gualzru.osg");
  osg::Node *osgMeshDST = osgDB::readNodeFile("../mesh/gualzru.osg");
  patsrc->addChild(osgMeshSRC);
  patdst->addChild(osgMeshDST);
  patdst->setAttitude(osg::Quat(osg::DegreesToRadians(0.0),osg::Vec3(0,0,1)));
  patsrc->setAttitude(osg::Quat(osg::DegreesToRadians(0.0),osg::Vec3(0,0,1)));
  this->addChild(patsrc);
  this->addChild(patdst);
}

osgWay::~osgWay()
{

}

void osgWay::setWay(const std::vector<point> &way)
{
  this->setAllChildrenOff();
  // float k = 1.3;
  float x, y;
  int j=0, jump = 2;
  if(way.size()>=MAX_POINTS)
    jump = ceil(way.size()/MAX_POINTS);
  for(unsigned int i = 0; i<way.size(); i+=jump)
  {
    y = ((way.at(i).x *(5.12*Kfactor/320)));
    x = ((way.at(i).y *(3.84*Kfactor/240)));
    auto pat = (osg::PositionAttitudeTransform *)this->getChild(j);
    pat->setPosition(osg::Vec3(x,y,-0.5f));
    j++;
  }
  numActivePats = j;
  active = true;
  auto patsrc = (osg::PositionAttitudeTransform *)this->getChild(100);
  auto patdst = (osg::PositionAttitudeTransform *)this->getChild(101);
  point psrc = way.front();
  point psrc2 = way.at(2*jump);
  point pdst2 = way.at(way.size()-jump);
  point pdst = way.back();
  
  patsrc->setPosition(osg::Vec3(psrc.y*(3.84*Kfactor/240)-10.24*Kfactor/2,psrc.x*(5.12*Kfactor/320)-7.68*Kfactor/2,-0.5f));
  patsrc->setScale(osg::Vec3(0.03,0.03,-0.03));
  patdst->setPosition(osg::Vec3(pdst.y*(3.84*Kfactor/240)-10.24*Kfactor/2,pdst.x*(5.12*Kfactor/320)-7.68*Kfactor/2,-0.5f));
  patdst->setScale(osg::Vec3(0.03,0.03,-0.03));
  
  std::vector<float> vecSrc = {psrc2.x-psrc.x,psrc2.y-psrc.y};
  double angleSrc = calculateRotation(vecSrc,(float)psrc.y);

  std::vector<float> vecDst = {pdst2.x-pdst.x,pdst2.y-pdst.y};
  double angleDst = calculateRotation(vecDst,(float)pdst.y);


  patdst->setAttitude(osg::Quat(osg::DegreesToRadians(180.)-angleDst,osg::Vec3(0,0,1)));
  patsrc->setAttitude(osg::Quat(osg::DegreesToRadians(180.)-angleSrc,osg::Vec3(0,0,1)));
  this->setValue(100,true);
  this->setValue(101,true);
}

double osgWay::calculateRotation(const std::vector<float> &Orientation,float axis)
{
	preVec[1] = axis;
	return acos((Orientation[0]*preVec[0]+Orientation[1]*preVec[1])/(sqrt(pow(preVec[0],2)+pow(preVec[1],2))*sqrt(pow(Orientation[0],2)+pow(Orientation[1],2))));
}

void osgWay::hide()
{
  this->setAllChildrenOff();
  active = false;
  numActivePats=0;
}

void osgWay::update()
{

  if(active)
  {
    for(int i=0; i<numActivePats; i++)
    {
      if (i%step==state){
        this->setValue(i,true);
      }else{
        this->setValue(i,false);
      }
    }
    state=(state+1)%step;
  }
}
