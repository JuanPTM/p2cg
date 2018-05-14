#ifndef _TYPES_H
#define _TYPES_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <limits>

typedef struct rgb_color
{
	uchar r;
	uchar g;
	uchar b;
	public:
	rgb_color(){
		r=255;
		g=255;
		b=255;
	}
	rgb_color(uchar r_, uchar g_, uchar b_){
		r=r_;
		g=g_;
		b=b_;
	};
} rgb_color;

typedef struct point
{
	int x;
	int y;
	int level;
	public:
	point(){
		x=0;
		y=0;
		level = -1;
	}
	point(int x_, int y_){
		x=x_;
		y=y_;
		level = -1;
	};
	point(int x_, int y_, int level_){
		x=x_;
		y=y_;
		level = level_;
	};
	bool operator==(const point& other) const
  {
		return (x==other.x && y==other.y);
	};
	friend std::ostream& operator<<(std::ostream& os, const point& p)
	{
    os << "point("<< p.x << ", " << p.y << ", " << p.level<<")";
    return os;
	}
	float adjacentDistance(const point& other,int value = -1) const
	{
		if (value == -1 || (abs(x-other.x) <= value && abs(y-other.y) <= value))
			return sqrt(pow(abs(x-other.x),2) + pow(abs(y-other.y),2) );
		else
			return std::numeric_limits<float>::max();
	};
	bool adjacent(const point& other,int value) const
	{
		return (abs(x-other.x) <= value && abs(y-other.y) <= value);
	};
	void setLevel(int level_)
	{
		level=level_;
	};
	int getLevel()
	{
		return level;
	};
} point;

#endif
