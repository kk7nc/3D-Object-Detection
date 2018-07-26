
#pragma once

#include <cmath>
#include <assert.h>

class vec3
{
public:
	float x, y, z;

	inline vec3()
	{
		x = y = z = 0.0f;
	}
	vec3(float e0, float e1, float e2) { x = e0; y = e1; z = e2; }

	vec3 operator*(const float &f) {
		return vec3(this->x*f, this->y*f, this->z*f);
	}
	vec3 operator/(const float &f) {
		return vec3(this->x / f, this->y / f, this->z / f);
	}

	vec3 &vec3::set(float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;


		return *this;
	}

	vec3 &vec3::zero(void) {
		x = y = z = 0;

		return *this;
	}

	vec3			operator- (void) const;
	vec3			operator+ (void) const;
	vec3			operator+ (const vec3 &v) const;
	vec3			operator- (const vec3 &v) const;
	vec3			operator* (float scalar) const;
	vec3			operator/ (float scalar) const;
	vec3 &			operator= (const vec3 &v);
	vec3 &			operator+=(const vec3 &v);
	vec3 &			operator-=(const vec3 &v);
	vec3 &			operator*=(float scalar);
	vec3 &			operator/=(float scalar);
	float			dot(const vec3 &v) const;
};