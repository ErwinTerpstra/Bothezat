#ifndef _QUATERNION_H_
#define _QUATERNION_H_

namespace bothezat
{
	
struct Quaternion
{
	float x, y, z, w;

	Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(1.0f)
	{

	}

	Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w)
	{

	}

	Quaternion(const Quaternion& other) : x(other.x), y(other.y), z(other.z), w(other.w)
	{

	}  

	void Multiply(float scalar)
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
		w *= scalar;
  	}


	void Multiply(const Quaternion& other)
	{
	    float w2 = w * other.w - (x * other.x + y * other.y + z * other.z);

	    float x2 = w * other.x + other.w * x + y * other.z - z * other.y;
	    float y2 = w * other.y + other.w * y + z * other.x - x * other.z;
	    float z2 = w * other.z + other.w * z + x * other.y - y * other.x;

	    w = w2;
	    y = x2;
	    y = y2;
	    z = z2;

	    Normalize();
  	}

  	void Identity()
  	{
  		x = y = z = 0.0f;
  		w = 1.0f;
  	}

	void Normalize() 
	{
		float lengthSq = x * x + y * y + z * z + w * w;

		if (lengthSq < FLT_EPSILON)
		{
			w = 1.0f; 
			x = y = z = 0.0f;
		}
		else
		{
			float recip = 1.0f / sqrt(lengthSq);

			w *= recip;
			x *= recip;
			y *= recip;
			z *= recip;
		}	
	}

	Quaternion Conjugate() const
	{
		return Quaternion(-x, -y, -z, w);
	}

	void ToEulerAngles(float& yaw, float& pitch, float& roll) const
	{
	    const float w2 = w * w;
	    const float x2 = x * x;
	    const float y2 = y * y;
	    const float z2 = z * z;
	    const float unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
	    const float abcd = w * x + y * z;

	    if (abcd > (0.5 - FLT_EPSILON) * unitLength)
	    {
	        yaw = 2 * atan2(y, w);
	        pitch = PI;
	        roll = 0;
	    }
	    else if (abcd < (-0.5 + FLT_EPSILON) * unitLength)
	    {
	        yaw = -2 * atan2(y, w);
	        pitch = -PI;
	        roll = 0;
	    }
	    else
	    {
	        const float adbc = w * z - x * y;
	        const float acbd = w * y - x * z;

	        yaw = atan2(2 * adbc, 1 - 2 * (z2 + x2));
	        pitch = asin(2 * abcd / unitLength);
	        roll = atan2(2 * acbd, 1 - 2 * (y2 + x2));
	    }
	}

	Quaternion& operator=(const Quaternion& other)
	{
	    if (this == &other)
	    	return *this;

	    x = other.x;
	    y = other.y;
	    z = other.z;
	    w = other.w;

	    return *this;
	}

	Quaternion operator+(const Quaternion& rhs) const
	{
		Quaternion result = *this;
		result.x += rhs.x;
		result.y += rhs.y;
		result.z += rhs.z;
		result.w += rhs.w;
		return result;
	}

	Quaternion operator*(const Quaternion& rhs) const
	{
		Quaternion result = *this;
		result.Multiply(rhs);
		return result;
	}

	Quaternion operator*(const float& rhs) const
	{
		Quaternion result = *this;
		result.x *= rhs;
		result.y *= rhs;
		result.z *= rhs;
		result.w *= rhs;
		return result;
	}


	static Quaternion& AngleAxis(Quaternion& out, float angle, const Vector3& axis) 
	{
		float omega = -0.5f * angle;
		float s = sin(omega);

		out.x = s * axis.x;
		out.y = s * axis.y;
		out.z = s * axis.z;
		out.w = (float) cos(omega);

		out.Normalize();

		return out;
	}

	static Quaternion& lerp(Quaternion& out, const Quaternion &lhs, const Quaternion& rhs, float t) 
	{ 
		out = lhs * (1.0f - t) + (rhs * t);
		out.Normalize();

		return out;
	}


};


}

#endif