

#include <osgview.h>

class Luz : public osg::PositionAttitudeTransform
{
  public:
    Luz(OsgView *osgw, int lightnum, osg::Vec4 pos, osg::Vec4 specular, osg::Vec4 diffuse, osg::Vec4 ambient);
    ~Luz();
    void move(float x, float y, float z);
    void switchLuz(bool state);
  private:
    osg::Light *light;
    OsgView *parent;

};
