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
  	}

  	void Identity()
  	{
  		x = y = z = 0.0f;
  		w = 1.0f;
  	}

	void Normalize() 
	{
		float lengthSq = sqrt(x * x + y * y + z * z + w * w);

		if (lengthSq == 0.0f)
		{
			w = 1.0f; 
			x = y = z = 0.0f;
		}
		else
		{
			float recip = 1.0f / lengthSq;

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


};

inline Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs)
{
	Quaternion result = lhs;
	result.Multiply(rhs);
	return result;
}

}

#endif