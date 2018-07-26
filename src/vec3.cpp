
#include "vec3.h"

#include <cmath>
#include <assert.h>

vec3 vec3::operator-(void) const {
	return vec3(-x, -y, -z);
}

vec3 vec3::operator+(void) const {
	return vec3(x, y, z);
}

vec3 vec3::operator+(const vec3 &v) const {
	return vec3(x + v.x, y + v.y, z + v.z);
}

vec3 vec3::operator-(const vec3 &v) const {
	return vec3(x - v.x, y - v.y, z - v.z);
}

vec3 vec3::operator*(float scalar) const {
	return vec3(x * scalar, y * scalar, z * scalar);
}

vec3 vec3::operator/(float scalar) const {
	float inv = 1.0f / scalar;
	return vec3(x * inv, y * inv, z * inv);
}

vec3 &vec3::operator=(const vec3 &v) {
	x = v.x;
	y = v.y;
	z = v.z;
	return *this;
}

vec3 &vec3::operator+=(const vec3 &v) {
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

vec3 &vec3::operator-=(const vec3 &v) {
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

vec3 &vec3::operator*=(float scalar) {
	x *= scalar;
	y *= scalar;
	z *= scalar;
	return *this;
}

vec3 &vec3::operator/=(float scalar) {
	float inv = 1.0f / scalar;
	x *= inv;
	y *= inv;
	z *= inv;
	return *this;
}

float vec3::dot(const vec3 &v) const {
	return x * v.x + y * v.y + z * v.z;
}
