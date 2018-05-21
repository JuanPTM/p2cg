#include "processfunction.h"




cv::Mat processfunction::FILTER(cv::Mat input)
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



std::vector< int >processfunction::dijk(int A, int B, const std::vector<point> points, int value)
{
	int n = points.size();
  vector< float > dist(n, std::numeric_limits<float>::max());
  vector< bool > vis(n, false);
	std::vector< std::vector< int > > result(n, std::vector<int>());
  dist.at(A) = 0;
	result.at(A).push_back(A);
	int j, i, cur;
	float path;
	point curP;
		for( i = 0; i < n; ++i)
		{
				cur = -1;
				for(j = 0; j < n; ++j)
				{
					if (vis[j]) continue;
					if (cur == -1 || dist[j] < dist[cur])
						cur = j;
				}
				vis[cur] = true;
				curP = points.at(cur);
			for(j = 0; j < n; ++j)
			{
				if(j == cur) continue;
				if(curP.adjacent(points.at(j), value))
				{
					path = dist.at(cur)+ curP.adjacentDistance(points.at(j), value);
					if (path < dist.at(j))
					{
						dist.at(j) = path;
						result.at(j).clear();
						result.at(j).insert(result.at(j).begin(), result.at(cur).begin(),result.at(cur).end());
						result.at(j).push_back(j);
					}
				}
			}
		}
	return result[B];
}

std::vector<point> processfunction::reduceListPoints(std::vector<point> pointsToWay, int value)
{
	std::vector<point> out;
	for(auto const p:pointsToWay)
		if(p.x%value==0 && p.y%value ==0)
			out.push_back(p);
	return out;
}


std::map<float,rgb_color> processfunction::loadPaleta()
{
	std::string key,r,g,b;
	ifstream readFile("../paleta.cfg");
	std::string line;
	std::map<float,rgb_color> paleta;
	while(std::getline(readFile,line))   {
    std::stringstream iss(line);
    std::getline(iss, key, '=');
    std::getline(iss, r, ',');
    std::getline(iss, g, ',');
    std::getline(iss, b, ',');
		paleta[std::stof(key)] = rgb_color(std::stoi(r),std::stoi(g),std::stoi(b));
	}
	readFile.close();
	return paleta;
}


rgb_color processfunction::ucharize(float v, std::map<float,rgb_color> paleta)
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

long processfunction::mymap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float processfunction::normalizeFloat(float v)
{
	if (v > 255)
		v = 255;
	if (v < 0)
		v = 0;
	return v;
}
