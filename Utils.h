#pragma once
class Utils
{
public:
	float dist(float x1, float y1, float z1, float x2, float y2, float z2);
	float randFloat();
	float random_2(float min, float max);//Totally random number within range
	float normalize2(float max, float value);
	float clamp(float val, float max, float min);
};

