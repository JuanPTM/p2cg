#ifndef _OSGWAY_H
#define _OSGWAY_H

#include <osgview.h>
#include <math.h>
#include "types/types.h"

#define MAX_POINTS 100


class osgWay: public  osg::Switch
{
private:
  int numActivePats;
  bool active;
  int state;
  int step;
  OsgView *osg;
public:
  osgWay(OsgView *osgw);
  ~osgWay();
  void setWay(const std::vector<point> &way);
  void hide();
  void update();
};


#endif
