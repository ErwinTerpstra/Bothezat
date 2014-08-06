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
		/*
	    float w2 = w * other.w - (x * other.x + y * other.y + z * other.z);

	    float x2 = w * other.x + other.w * x + y * other.z - z * other.y;
	    float y2 = w * other.y + other.w * y + z * other.x - x * other.z;
	    float z2 = w * other.z + other.w * z + x * other.y - y * other.x;
		/*/
	    const Quaternion& q1 = *this;
	    const Quaternion& q2 = other;
	    float x2 =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
	    float y2 = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
	    float z2 =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
	    float w2 = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
	    //*/

	    w = w2;
	    x = x2;
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
		float lengthSq = x * x + y * y + z * z + w * w;

		if (lengthSq < FLT_EPSILON)
			Identity();
		else
			Multiply(1.0f / sqrt(lengthSq));
	}

	Quaternion Conjugate() const
	{
		return Quaternion(-x, -y, -z, w);
	}

	void ToEulerAngles(float& yaw, float& pitch, float& roll) const
	{
	    float w2 = w * w;
	    float x2 = x * x;
	    float y2 = y * y;
	    float z2 = z * z;
	    


	    //float abcd = x * y + z * w; // euclideanspace.com
	    //float abcd = y * w - x * z; // Wikipedia
	    float abcd = y * z + w * x;

	    if (Util::Approximately(abcd, 0.5f))
	    {
	        yaw = 2 * atan2(x, w);
	        pitch = PI_HALF;
	        roll = 0;
	    }
	    else if (Util::Approximately(abcd, -0.5f))
	    {
	        yaw = -2 * atan2(x, w);
	        pitch = -PI_HALF;
	        roll = 0;
	    }
	    else
	    {
			// euclideanspace.com
            //yaw = atan2(2 * w * y - 2 * x * z, 1 - 2 * (z2 + y2));
            //pitch = asin(2 * abcd);
            //roll = atan2(2 * w * x - 2 * y * z, 1 - 2 * (x2 + z2));	    	

            // Wikipedia
            //yaw = atan2(2 * (w * y + x * z), 1 - 2 * (x2 + y2));
            //pitch = asin(2 * abcd);
            //roll = atan2(2 * (w * z + x * y), 1 - 2 * (y2 + z2));

            yaw = atan2(x * z + w * y, 0.5f - (x2 + y2));
            pitch = asin(-2 * abcd);
            roll = atan2(x * y + w * z, 0.5f - (y2 + z2));
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

	Quaternion operator-(const Quaternion& rhs) const
	{
		Quaternion result = *this;
		result.x -= rhs.x;
		result.y -= rhs.y;
		result.z -= rhs.z;
		result.w -= rhs.w;

		return result;
	}

	Quaternion operator*(const Quaternion& rhs) const
	{
		Quaternion result = *this;
		result.Multiply(rhs);

		return result;
	}

	Vector3 operator*(const Vector3& rhs) const
	{
		Vector3 v(x, y, z);
		Vector3 t = Vector3::Cross(v, rhs) * 2.0f;
		return rhs + t * w + Vector3::Cross(v, t);
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

	Quaternion operator-() const
	{
		return Quaternion(-x, -y, -z, -w);
	}

	bool IsValid() const
	{
		return !(isnan(x) || isnan(y) || isnan(z) || isnan(w));
	}

	void Print() const
	{
		Debug::Print("%.4f;%.4f;%.4f;%.4f\n", x, y, z, w);
	}

	static float Dot(const Quaternion& q1, const Quaternion& q2) 
	{
		return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
	}

	static Quaternion AngleAxis(float angle, const Vector3& axis) 
	{
		Quaternion result;
		AngleAxis(angle, axis);

		return result;
	}

	static Quaternion& AngleAxis(Quaternion& out, float angle, const Vector3& axis) 
	{
		float omega = 0.5f * angle;
		float s = sin(omega);

		out.x = s * axis.x;
		out.y = s * axis.y;
		out.z = s * axis.z;
		out.w = cos(omega);

		out.Normalize();

		return out;
	}

	static Quaternion Lerp(const Quaternion& lhs, const Quaternion& rhs, float t) 
	{
		Quaternion result;
		Lerp(result, lhs, rhs, t);

		return result;
	}

	static Quaternion& Lerp(Quaternion& out, const Quaternion &lhs, const Quaternion& rhs, float t) 
	{ 
		out = lhs * (1.0f - t) + (rhs * t);
		out.Normalize();

		return out;
	}

	static Quaternion& Slerp(const Quaternion& q1, const Quaternion& q2, float t)
	{
		Quaternion result;
		Slerp(result, q1, q2, t);

		return result;
	}

	static Quaternion& Slerp(Quaternion& out, const Quaternion& q1, const Quaternion& q2, float t) 
	{
		Quaternion q3;
		float dot = Dot(q1, q2);

		/*	dot = cos(theta)
			if (dot < 0), q1 and q2 are more than 90 degrees apart,
			so we can invert one to reduce spinning	*/
		if (dot < 0)
		{
			dot = -dot;
			q3 = -q2;
		} else 
			q3 = q2;
		
		if (dot < 0.9995f)
		{
			float angle = acos(dot);
			float theta = angle * t;

		    q3 = q3 - q1 * dot;
		    q3.Normalize();

		    out = q1 * cos(theta) + q3 * sin(theta);
		    out.Normalize();
		}
		else
			Lerp(out, q1, q3, t);

		return out;
	}

	static Quaternion LookAt(const Vector3& forward, const Vector3& up)
	{
		Quaternion result;
		LookAt(result, forward, up);

		return result;
	}

	static Quaternion& LookAt(Quaternion& out, const Vector3& forward, const Vector3& up)
	{
		// right.x 		up.x 	forward.x
		// right.y 		up.y 	forward.y
		// right.z 		up.z 	forward.z
		Vector3 right = Vector3::Cross(forward, up);

		float m00 = right.x, m01 = right.y, m02 = right.z,
			  m10 = up.x, m11 = up.y, m12 = up.z,
			  m20 = forward.x, m21 = forward.y, m22 = forward.z;

		float tr = right.x + up.y + forward.z;

		if (tr > 0) 
		{	
			out.w = sqrt(1.0f + tr) * 0.5f;
			float recipW = 1.0f / (4.0f * out.w);

			out.x = (m21 - m12) * recipW;
			out.y = (m02 - m20) * recipW; 
			out.z = (m10 - m01) * recipW; 
		}
		else if ((m00 > m11) && (m00 > m22))
		{ 
			out.x = sqrt(1.0f + m00 - m11 - m22) * 0.5f;
			float recipX = 1.0f / (4.0f * out.x);

			out.w = (m21 - m12) * recipX;
			out.y = (m01 + m10) * recipX; 
			out.z = (m02 + m20) * recipX; 
		}
		else if (m11 > m22) 
		{
			out.y = sqrt(1.0f + m11 - m00 - m22) * 0.5f; 
			float recipY = 1.0f / (4.0f * out.y);

			out.w = (m02 - m20) * recipY;
			out.x = (m01 + m10) * recipY; 
			out.z = (m12 + m21) * recipY; 
		}
		else
		{ 
			out.z = sqrt(1.0f + m22 - m00 - m11) * 0.5f; 
			float recipZ = 1.0f / (4.0f * out.z);

			out.w = (m10 - m01) * recipZ;
			out.x = (m02 + m20) * recipZ;
			out.y = (m12 + m21) * recipZ;
		}

		out.Normalize();

		return out;
	}

	static Quaternion RotationBetween(const Vector3& u, const Vector3& v)
	{
		Quaternion result;
		RotationBetween(result, u, v);

		return result;
	}

	static Quaternion& RotationBetween(Quaternion& out, const Vector3& u, const Vector3& v)
	{
	    float magnitude = sqrt(u.LengthSq() * v.LengthSq());
	    Vector3 w = Vector3::Cross(u, v);

	    out.w = magnitude + Vector3::Dot(u, v);
	    out.x = w.x;
	    out.y = w.y;
	    out.z = w.z;
	    out.Normalize();

	    return out;
	}

};

}

#endif